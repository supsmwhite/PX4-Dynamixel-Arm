#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

struct DxlStatusPacket {
	uint8_t id;
	uint8_t error;
	uint8_t params[32];
	size_t param_len;
};

// Dynamixel Protocol 2.0 标准 CRC16 (多项式 0x8005)
static uint16_t dxl_calc_crc(const uint8_t *data, size_t length)
{
	uint16_t crc = 0;

	for (size_t j = 0; j < length; j++) {
		crc ^= (uint16_t)data[j] << 8;

		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (uint16_t)((crc << 1) ^ 0x8005);

			} else {
				crc = (uint16_t)(crc << 1);
			}
		}
	}

	return crc;
}

// 严格解析舵机目标位置参数 (0 ~ 4095)，拒绝任何非数字字符或溢出输入
static bool parse_goal_pos(const char *str, int32_t *out_pos)
{
	if (str == nullptr || *str == '\0') {
		return false;
	}

	errno = 0;
	char *endptr = nullptr;
	long val = strtol(str, &endptr, 10);

	if (errno != 0 || endptr == str || *endptr != '\0') {
		return false;
	}

	if (val < 0 || val > 4095) {
		return false;
	}

	*out_pos = (int32_t)val;
	return true;
}

// 打开并配置 NuttX 串口为纯二进制 Raw 模式
static int dxl_open_port(const char *dev, int baud)
{
	int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("无法打开串口 %s (errno=%d)", dev, errno);
		return -1;
	}

	struct termios t{};

	if (tcgetattr(fd, &t) != 0) {
		PX4_ERR("tcgetattr 失败 (errno=%d)", errno);
		::close(fd);
		return -1;
	}

	t.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
	t.c_oflag &= ~OPOST;
	t.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	t.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
	t.c_cflag |= (CS8 | CREAD | CLOCAL);

	speed_t sp = B57600;

	if (baud == 1000000) {
		sp = B1000000;

	} else if (baud == 115200) {
		sp = B115200;
	}

	if (cfsetispeed(&t, sp) != 0 || cfsetospeed(&t, sp) != 0) {
		PX4_ERR("设置波特率失败");
		::close(fd);
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		PX4_ERR("tcsetattr 失败 (errno=%d)", errno);
		::close(fd);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);
	return fd;
}

// 完整写入所有字节，妥善处理非阻塞串口短写、EINTR 与 EAGAIN
static bool dxl_write_all(int fd, const uint8_t *buf, size_t len, int timeout_ms)
{
	size_t total_written = 0;
	hrt_abstime start_time = hrt_absolute_time();
	uint64_t timeout_us = (uint64_t)timeout_ms * 1000ULL;

	while (total_written < len) {
		ssize_t w = ::write(fd, buf + total_written, len - total_written);

		if (w > 0) {
			total_written += (size_t)w;
			continue;
		}

		if (w < 0 && errno == EINTR) {
			continue;
		}

		if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			uint64_t elapsed_us = hrt_elapsed_time(&start_time);

			if (elapsed_us >= timeout_us) {
				PX4_ERR("串口写入超时: 已写 %u / %u", (unsigned)total_written, (unsigned)len);
				return false;
			}

			struct pollfd pfd = { .fd = fd, .events = POLLOUT, .revents = 0 };
			int remain_ms = (int)((timeout_us - elapsed_us) / 1000ULL);

			if (remain_ms < 1) {
				remain_ms = 1;
			}

			int pret = ::poll(&pfd, 1, remain_ms);

			if (pret < 0 && errno != EINTR) {
				PX4_ERR("串口写入 poll 失败 (errno=%d)", errno);
				return false;
			}

			continue;
		}

		PX4_ERR("串口写入失败: ret=%d, errno=%d", (int)w, errno);
		return false;
	}

	return true;
}

// 构建 Protocol 2.0 指令帧 (支持 FF FF FD 字节填充并自动更新 Length 与 CRC16)
static int dxl_build_packet(uint8_t id, uint8_t inst, const uint8_t *params, size_t param_len,
			    uint8_t *out_buf, size_t out_max)
{
	if (out_max < 10) {
		return -1;
	}

	out_buf[0] = 0xFF;
	out_buf[1] = 0xFF;
	out_buf[2] = 0xFD;
	out_buf[3] = 0x00;
	out_buf[4] = id;

	size_t idx = 7;
	uint8_t ff_cnt = 0;
	size_t raw_len = 1 + param_len;

	for (size_t m = 0; m < raw_len; m++) {
		uint8_t b = (m == 0) ? inst : params[m - 1];

		if (idx + 2 > out_max) {
			return -1;
		}

		out_buf[idx++] = b;

		if (b == 0xFF) {
			if (ff_cnt < 2) {
				ff_cnt++;
			}

		} else if (b == 0xFD && ff_cnt == 2) {
			if (idx + 2 > out_max) {
				return -1;
			}

			out_buf[idx++] = 0xFD; // 插入协议字节填充
			ff_cnt = 0;

		} else {
			ff_cnt = 0;
		}
	}

	uint16_t pkt_len = (uint16_t)((idx - 7) + 2);
	out_buf[5] = (uint8_t)(pkt_len & 0xFF);
	out_buf[6] = (uint8_t)((pkt_len >> 8) & 0xFF);

	uint16_t crc = dxl_calc_crc(out_buf, idx);
	out_buf[idx++] = (uint8_t)(crc & 0xFF);
	out_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

	return (int)idx;
}

// 按报文 Length 字段取完整帧 -> 校验 CRC16 -> 去除 FF FF FD FD 字节填充
static int dxl_parse_frames(const uint8_t *rx, size_t rx_len, DxlStatusPacket *out_pkts,
			    int max_pkts, bool resync_incomplete)
{
	int count = 0;
	size_t i = 0;

	while (i + 7 <= rx_len && count < max_pkts) {
		if (rx[i] != 0xFF || rx[i + 1] != 0xFF || rx[i + 2] != 0xFD || rx[i + 3] != 0x00) {
			i++;
			continue;
		}

		uint8_t pkt_id = rx[i + 4];
		uint16_t pkt_len = (uint16_t)((uint16_t)rx[i + 5] | ((uint16_t)rx[i + 6] << 8));

		if (pkt_len < 3 || pkt_len > 128) {
			i++;
			continue;
		}

		size_t frame_len = 7 + (size_t)pkt_len;

		if (i + frame_len > rx_len) {
			if (resync_incomplete) {
				i++;
				continue;
			}

			break; // 仍在接收窗口内，等待后续字节补齐整帧
		}

		uint16_t rx_crc = (uint16_t)((uint16_t)rx[i + frame_len - 2] |
					     ((uint16_t)rx[i + frame_len - 1] << 8));
		uint16_t calc_crc = dxl_calc_crc(&rx[i], frame_len - 2);

		if (rx_crc != calc_crc) {
			i++;
			continue;
		}

		if (rx[i + 7] == 0x55 && pkt_len >= 4) {
			const uint8_t *stuffed = &rx[i + 7];
			size_t stuffed_len = (size_t)pkt_len - 2;
			uint8_t unstuffed[128]{};
			size_t unstuffed_len = 0;

			for (size_t k = 0; k < stuffed_len && unstuffed_len < sizeof(unstuffed); k++) {
				if (k + 3 < stuffed_len &&
				    stuffed[k] == 0xFF && stuffed[k + 1] == 0xFF &&
				    stuffed[k + 2] == 0xFD && stuffed[k + 3] == 0xFD) {
					if (unstuffed_len + 3 > sizeof(unstuffed)) {
						break;
					}

					unstuffed[unstuffed_len++] = 0xFF;
					unstuffed[unstuffed_len++] = 0xFF;
					unstuffed[unstuffed_len++] = 0xFD;
					k += 3;

				} else {
					unstuffed[unstuffed_len++] = stuffed[k];
				}
			}

			if (unstuffed_len >= 2 && unstuffed[0] == 0x55) {
				size_t p_len = unstuffed_len - 2;

				if (p_len <= sizeof(out_pkts[count].params)) {
					out_pkts[count].id = pkt_id;
					out_pkts[count].error = unstuffed[1];
					out_pkts[count].param_len = p_len;

					if (p_len > 0) {
						memcpy(out_pkts[count].params, &unstuffed[2], p_len);
					}

					count++;
				}
			}
		}

		// 整帧跳过 (自动过滤微雪板半双工自回显的合法指令帧)
		i += frame_len;
	}

	return count;
}

// 发送请求帧并读取解析指定数量的 Status Packet (收齐即提前返回)
static int dxl_exchange_status(int fd, const uint8_t *tx_buf, size_t tx_len,
			       DxlStatusPacket *out_pkts, int expected_pkts, int timeout_ms, size_t *out_raw_bytes)
{
	tcflush(fd, TCIFLUSH);

	if (!dxl_write_all(fd, tx_buf, tx_len, timeout_ms)) {
		return -1;
	}

	uint8_t rx_buf[256]{};
	size_t total_rx = 0;
	struct pollfd fds[1] = {{ .fd = fd, .events = POLLIN, .revents = 0 }};
	hrt_abstime start_time = hrt_absolute_time();
	uint64_t timeout_us = (uint64_t)timeout_ms * 1000ULL;

	while (total_rx < sizeof(rx_buf)) {
		uint64_t elapsed_us = hrt_elapsed_time(&start_time);

		if (elapsed_us >= timeout_us) {
			break;
		}

		int remain_ms = (int)((timeout_us - elapsed_us) / 1000ULL);

		if (remain_ms < 1) {
			remain_ms = 1;
		}

		int ret = ::poll(fds, 1, remain_ms);

		if (ret < 0) {
			if (errno == EINTR) {
				continue;
			}

			PX4_ERR("串口读取 poll 失败 (errno=%d)", errno);
			return -1;
		}

		if (ret == 0) {
			continue;
		}

		if (fds[0].revents & POLLIN) {
			ssize_t n = ::read(fd, rx_buf + total_rx, sizeof(rx_buf) - total_rx);

			if (n > 0) {
				total_rx += (size_t)n;
				int got = dxl_parse_frames(rx_buf, total_rx, out_pkts, expected_pkts, false);

				if (got >= expected_pkts) {
					if (out_raw_bytes != nullptr) {
						*out_raw_bytes = total_rx;
					}

					return got;
				}

			} else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
				PX4_ERR("串口 read 失败 (errno=%d)", errno);
				return -1;
			}
		}
	}

	if (out_raw_bytes != nullptr) {
		*out_raw_bytes = total_rx;
	}

	return dxl_parse_frames(rx_buf, total_rx, out_pkts, expected_pkts, true);
}

// 同步读取 ID 1, 2, 3 的 4 字节整型寄存器 (用于读当前位置 Addr 132 或读回校验目标位置 Addr 116)
static bool dxl_sync_read_i32(int fd, uint16_t addr, int32_t out_vals[3], uint8_t out_errs[3], size_t *out_raw_bytes)
{
	uint8_t sr_params[7] = {
		(uint8_t)(addr & 0xFF), (uint8_t)((addr >> 8) & 0xFF),
		0x04, 0x00,
		1, 2, 3
	};
	uint8_t tx[32]{};
	int tx_len = dxl_build_packet(0xFE, 0x82, sr_params, sizeof(sr_params), tx, sizeof(tx));

	if (tx_len < 0) {
		return false;
	}

	DxlStatusPacket pkts[3]{};
	int got = dxl_exchange_status(fd, tx, (size_t)tx_len, pkts, 3, 60, out_raw_bytes);

	if (got < 0) {
		return false;
	}

	bool seen[3] = {false, false, false};

	for (int idx = 0; idx < got; idx++) {
		uint8_t id = pkts[idx].id;

		if (id >= 1 && id <= 3 && pkts[idx].param_len == 4) {
			uint32_t raw = (uint32_t)pkts[idx].params[0] |
				       ((uint32_t)pkts[idx].params[1] << 8) |
				       ((uint32_t)pkts[idx].params[2] << 16) |
				       ((uint32_t)pkts[idx].params[3] << 24);
			out_vals[id - 1] = (int32_t)raw;
			out_errs[id - 1] = pkts[idx].error;
			seen[id - 1] = true;
		}
	}

	return seen[0] && seen[1] && seen[2];
}

extern "C" __EXPORT int dxl_arm_main(int argc, char *argv[])
{
	const char *dev = "/dev/ttyS5"; // TELEM1
	int baud = 57600;

	if (argc < 2) {
		PX4_INFO("用法: dxl_arm [ping | move <pos1> <pos2> <pos3> | read]");
		return -1;
	}

	if (strcmp(argv[1], "ping") == 0) {
		int fd = dxl_open_port(dev, baud);

		if (fd < 0) {
			return -1;
		}

		bool all_ok = true;

		for (uint8_t id = 1; id <= 3; id++) {
			uint8_t tx[16]{};
			int tx_len = dxl_build_packet(id, 0x01, nullptr, 0, tx, sizeof(tx));
			DxlStatusPacket pkt{};
			size_t raw_bytes = 0;

			int got = dxl_exchange_status(fd, tx, (size_t)tx_len, &pkt, 1, 50, &raw_bytes);

			if (got >= 1 && pkt.id == id && pkt.param_len >= 3) {
				uint16_t model = (uint16_t)((uint16_t)pkt.params[0] | ((uint16_t)pkt.params[1] << 8));
				PX4_INFO("Ping ID %u: [在线] 型号=%u, 固件=v%u, 错误码=0x%02X (串口共收 %u 字节)",
					 (unsigned)id, (unsigned)model, (unsigned)pkt.params[2],
					 (unsigned)pkt.error, (unsigned)raw_bytes);

			} else {
				PX4_WARN("Ping ID %u: [未响应] (串口共收 %u 字节)", (unsigned)id, (unsigned)raw_bytes);
				all_ok = false;
			}
		}

		::close(fd);
		return all_ok ? 0 : -1;

	} else if (strcmp(argv[1], "move") == 0) {
		if (argc != 5) {
			PX4_ERR("参数数量错误! 用法: dxl_arm move <pos1> <pos2> <pos3>");
			return -1;
		}

		int32_t pos[3]{};

		for (int m = 0; m < 3; m++) {
			if (!parse_goal_pos(argv[m + 2], &pos[m])) {
				PX4_ERR("第 %d 个目标位置参数非法 (%s)! 请输入 0 ~ 4095 之间的整数", m + 1, argv[m + 2]);
				return -1;
			}
		}

		int fd = dxl_open_port(dev, baud);

		if (fd < 0) {
			return -1;
		}

		// 1. 广播开启 3 台电机力矩 (Addr 64 = 1)
		uint8_t tq_params[3] = {64, 0x00, 0x01};
		uint8_t tq_tx[24]{};
		int tq_len = dxl_build_packet(0xFE, 0x03, tq_params, sizeof(tq_params), tq_tx, sizeof(tq_tx));

		if (tq_len < 0 || !dxl_write_all(fd, tq_tx, (size_t)tq_len, 50)) {
			PX4_ERR("开启电机力矩发送失败");
			::close(fd);
			return -1;
		}

		px4_usleep(10000); // 等待 57600 波特率下报文发完且电机完成力矩使能

		// 2. Sync Write (0x83) 同步写入 3 台电机目标位置 (Addr 116, 4字节)
		uint8_t sw_params[19] = {
			116, 0x00, 0x04, 0x00,
			1, (uint8_t)(pos[0] & 0xFF), (uint8_t)((pos[0] >> 8) & 0xFF), (uint8_t)((pos[0] >> 16) & 0xFF), (uint8_t)((pos[0] >> 24) & 0xFF),
			2, (uint8_t)(pos[1] & 0xFF), (uint8_t)((pos[1] >> 8) & 0xFF), (uint8_t)((pos[1] >> 16) & 0xFF), (uint8_t)((pos[1] >> 24) & 0xFF),
			3, (uint8_t)(pos[2] & 0xFF), (uint8_t)((pos[2] >> 8) & 0xFF), (uint8_t)((pos[2] >> 16) & 0xFF), (uint8_t)((pos[2] >> 24) & 0xFF)
		};
		uint8_t sw_tx[48]{};
		int sw_len = dxl_build_packet(0xFE, 0x83, sw_params, sizeof(sw_params), sw_tx, sizeof(sw_tx));

		if (sw_len < 0 || !dxl_write_all(fd, sw_tx, (size_t)sw_len, 50)) {
			PX4_ERR("同步写入目标位置发送失败");
			::close(fd);
			return -1;
		}

		px4_usleep(10000); // 等待 29 字节在线路上传输完毕并写入电机寄存器

		// 3. 读回 Addr 116 (Goal Position) 确认 3 台电机真实在线、无告警且目标位置已生效
		int32_t rb_pos[3]{};
		uint8_t rb_err[3]{};

		if (!dxl_sync_read_i32(fd, 116, rb_pos, rb_err, nullptr)) {
			PX4_ERR("运动指令读回校验失败: 未收齐 3 台电机响应，请检查供电与 TX/RX 接线");
			::close(fd);
			return -1;
		}

		for (int m = 0; m < 3; m++) {
			if (rb_err[m] != 0 || rb_pos[m] != pos[m]) {
				PX4_ERR("电机 ID %d 目标位置未正常生效 (期望=%ld, 读回=%ld, 错误码=0x%02X)",
					m + 1, (long)pos[m], (long)rb_pos[m], (unsigned)rb_err[m]);
				::close(fd);
				return -1;
			}
		}

		PX4_INFO("已下发并读回确认目标位置: ID1=%ld, ID2=%ld, ID3=%ld",
			 (long)pos[0], (long)pos[1], (long)pos[2]);
		::close(fd);
		return 0;

	} else if (strcmp(argv[1], "read") == 0) {
		int fd = dxl_open_port(dev, baud);

		if (fd < 0) {
			return -1;
		}

		int32_t cur_pos[3]{};
		uint8_t cur_err[3]{};
		size_t raw_bytes = 0;
		bool ok = dxl_sync_read_i32(fd, 132, cur_pos, cur_err, &raw_bytes);

		if (!ok) {
			PX4_ERR("Sync Read 未收齐 3 台电机状态 (串口共收 %u 字节)", (unsigned)raw_bytes);
			::close(fd);
			return -1;
		}

		for (int m = 0; m < 3; m++) {
			PX4_INFO("电机 ID %d -> 当前位置: %ld (错误码: 0x%02X)",
				 m + 1, (long)cur_pos[m], (unsigned)cur_err[m]);
		}

		::close(fd);
		return 0;
	}

	PX4_ERR("未知子命令: %s! 用法: dxl_arm [ping | move <pos1> <pos2> <pos3> | read]", argv[1]);
	return -1;
}
