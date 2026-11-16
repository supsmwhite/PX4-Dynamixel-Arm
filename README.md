# PX4 DYNAMIXEL 机械臂串口适配

本仓库在 PX4 官方 `release/1.18` 分支基础上，为 **CUAV 7-Nano** 新增 `dxl_arm` 驱动，通过 **TELEM1 串口控制 ID 1、2、3 三台 DYNAMIXEL 舵机**。当前已在作者的硬件台架上实现控制效果，供交流与复现。

- **已实现**：舵机在线检测、三台舵机同步目标位置写入、目标寄存器读回确认、当前位置读取；通过 PX4 的 NSH 控制台调用。
- **主要改动**：新增 [`src/drivers/dxl_arm/`](src/drivers/dxl_arm/)，并在 [CUAV 7-Nano 板级配置](boards/cuav/7-nano/default.px4board)中启用驱动。协议处理包含 CRC16、字节填充、接收解析和串口写入错误处理。
- **从这里开始**：阅读 [驱动说明：改动、配置与运行方法](src/drivers/dxl_arm/README.md)，或直接[查看相对官方基线的全部改动](https://github.com/supsmwhite/PX4-Dynamixel-Arm/compare/8303efda43...dxl-arm)。

下面保留 PX4 官方项目介绍；本项目的操作说明以上方链接为准。

---

<p align="center">
  <a href="https://px4.io">
    <img src="docs/assets/site/px4_logo.svg" alt="PX4 Autopilot" width="240">
  </a>
</p>

<p align="center">
  <em>The autopilot stack the industry builds on.</em>
</p>

<p align="center">
  <a href="https://github.com/PX4/PX4-Autopilot/releases"><img src="https://img.shields.io/github/release/PX4/PX4-Autopilot.svg" alt="Release"></a>
  <a href="https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot"><img src="https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg" alt="DOI"></a>
  <a href="https://discord.gg/dronecode"><img src="https://img.shields.io/discord/1022170275984457759?label=discord&logo=discord&logoColor=white&color=5865F2" alt="Discord"></a>
</p>

<p align="center">
  <a href="https://www.bestpractices.dev/projects/6520"><img src="https://www.bestpractices.dev/projects/6520/badge" alt="OpenSSF Best Practices"></a>
  <a href="https://insights.linuxfoundation.org/project/px4"><img src="https://insights.linuxfoundation.org/api/badge/health-score?project=px4" alt="LFX Health Score"></a>
  <a href="https://insights.linuxfoundation.org/project/px4"><img src="https://insights.linuxfoundation.org/api/badge/contributors?project=px4" alt="LFX Contributors"></a>
  <a href="https://insights.linuxfoundation.org/project/px4"><img src="https://insights.linuxfoundation.org/api/badge/active-contributors?project=px4" alt="LFX Active Contributors"></a>
</p>

---

## About

PX4 is an open-source autopilot stack for drones and unmanned vehicles. It supports multirotors, fixed-wing, VTOL, rovers, and many more experimental platforms from racing quads to industrial survey aircraft. It runs on [NuttX](https://nuttx.apache.org/), Linux, and macOS. Licensed under [BSD 3-Clause](LICENSE).

## Why PX4

**Modular architecture.** PX4 is built around [uORB](https://docs.px4.io/main/en/middleware/uorb.html), a [DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html)-compatible publish/subscribe middleware. Modules are fully parallelized and thread safe. You can build custom configurations and trim what you don't need.

**Wide hardware support.** PX4 runs on a wide range of [autopilot boards](https://docs.px4.io/main/en/flight_controller/) and supports an extensive set of sensors, telemetry radios, and actuators through the [Pixhawk](https://pixhawk.org/) ecosystem.

**Developer friendly.** First-class support for [MAVLink](https://mavlink.io/) and [DDS / ROS 2](https://docs.px4.io/main/en/ros2/) integration. Comprehensive [SITL simulation](https://docs.px4.io/main/en/simulation/), hardware-in-the-loop testing, and [log analysis](https://docs.px4.io/main/en/log/flight_log_analysis.html) tools. An active developer community on [Discord](https://discord.gg/dronecode) and the [weekly dev call](https://docs.px4.io/main/en/contribute/).

**Vendor neutral governance.** PX4 is hosted under the [Dronecode Foundation](https://www.dronecode.org/), part of the Linux Foundation. Business-friendly BSD-3 license. No single vendor controls the roadmap.

## Supported Vehicles

<table>
  <tr>
    <td align="center">
      <a href="https://docs.px4.io/main/en/frames_multicopter/">
        <img src="docs/assets/airframes/types/QuadRotorX.svg" width="50" alt="Multicopter"><br>
        <sub>Multicopter</sub>
      </a>
    </td>
    <td align="center">
      <a href="https://docs.px4.io/main/en/frames_plane/">
        <img src="docs/assets/airframes/types/Plane.svg" width="50" alt="Fixed Wing"><br>
        <sub>Fixed Wing</sub>
      </a>
    </td>
    <td align="center">
      <a href="https://docs.px4.io/main/en/frames_vtol/">
        <img src="docs/assets/airframes/types/VTOLPlane.svg" width="50" alt="VTOL"><br>
        <sub>VTOL</sub>
      </a>
    </td>
    <td align="center">
      <a href="https://docs.px4.io/main/en/frames_rover/">
        <img src="docs/assets/airframes/types/Rover.svg" width="50" alt="Rover"><br>
        <sub>Rover</sub>
      </a>
    </td>
  </tr>
</table>

<sub>…and many more: helicopters, autogyros, airships, submarines, boats, and other experimental platforms. These frames have basic support but are not part of the regular flight-test program. See the <a href="https://docs.px4.io/main/en/airframes/airframe_reference.html">full airframe reference</a>.</sub>

## Try PX4

Run PX4 in simulation with a single command. No build tools, no dependencies beyond Docker:

```bash
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl:latest
```

Open [QGroundControl](https://qgroundcontrol.com) and fly. See [PX4 Simulation Quickstart](https://docs.px4.io/main/en/simulation/px4_simulation_quickstart) for more options.

## Build from Source

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl
```

> [!NOTE]
> See the [Development Guide](https://docs.px4.io/main/en/development/development.html) for toolchain setup and build options.

## Documentation & Resources

| Resource | Description |
| --- | --- |
| [User Guide](https://docs.px4.io/main/en/) | Build, configure, and fly with PX4 |
| [Developer Guide](https://docs.px4.io/main/en/development/development.html) | Modify the flight stack, add peripherals, port to new hardware |
| [Airframe Reference](https://docs.px4.io/main/en/airframes/airframe_reference.html) | Full list of supported frames |
| [Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/) | Compatible flight controllers |
| [Release Notes](https://docs.px4.io/main/en/releases/) | What's new in each release |
| [Contribution Guide](https://docs.px4.io/main/en/contribute/) | How to contribute to PX4 |

## Community

- **Weekly Dev Call** — open to all developers ([Dronecode calendar](https://www.dronecode.org/calendar/))
- **Discord** — [Join the Dronecode server](https://discord.gg/dronecode)
- **Discussion Forum** — [PX4 Discuss](https://discuss.px4.io/)
- **Maintainers** — see [`MAINTAINERS.md`](MAINTAINERS.md)
- **Contributor Stats** — [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode)

## Contributing

We welcome contributions of all kinds — bug reports, documentation, new features, and code reviews. Please read the [Contribution Guide](https://docs.px4.io/main/en/contribute/) to get started.

## Citation

If you use PX4 in academic work, please cite it. BibTeX:

```bibtex
@software{px4_autopilot,
  author    = {Meier, Lorenz and {The PX4 Contributors}},
  title     = {{PX4 Autopilot}},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.595432},
  url       = {https://px4.io}
}
```

The DOI above is a Zenodo concept DOI that always resolves to the latest release. For a version-pinned citation, see the [Zenodo record](https://doi.org/10.5281/zenodo.595432) or our [`CITATION.cff`](CITATION.cff).

## Governance

The PX4 Autopilot project is hosted by the [Dronecode Foundation](https://www.dronecode.org/), a [Linux Foundation](https://www.linuxfoundation.org/) Collaborative Project. Dronecode holds all PX4 trademarks and serves as the project's legal guardian, ensuring vendor-neutral stewardship — no single company owns the name or controls the roadmap. The source code is licensed under the [BSD 3-Clause](LICENSE) license, so you are free to use, modify, and distribute it in your own projects.

<p align="center">
  <a href="https://www.dronecode.org/">
    <img src="docs/assets/site/dronecode_logo.svg" alt="Dronecode Logo" width="180">
  </a>
</p>
