"""
| File: ardupilot_launch_tool.py
| Author: Tomer Tip (tomerT1212@gmail.com)
| Description: Defines an auxiliary tool to launch the Ardupilot process in the background
| License: BSD-3-Clause. Copyright (c) 2024, Tomer Tip. All rights reserved.
"""

import os
import shlex
import shutil
import signal
import tempfile
import time
import subprocess
from pathlib import Path
from typing import Optional, Tuple


class ArduPilotLaunchTool:
    """
    A class that manages the start/stop of a ardupilot process. It requires only the path to the Ardupilot installation (assuming that
    Ardupilot was already built with 'make ardupilot_sitl_default none'), the vehicle id and the vehicle model.
    """

    def __init__(
        self,
        ardupilot_dir,
        vehicle_id: int = 0,
        ardupilot_model: str = "gazebo-iris",
        mavlink_out_host: str = "127.0.0.1",
        mavlink_udp_port: int = 14580,
    ):
        """Construct the ArduPilotLaunchTool object

        Args:
            ardupilot_dir (str): A string with the path to the Ardupilot-Autopilot directory
            vehicle_id (int): The ID of the vehicle. Defaults to 0.
            ardupilot_model (str): The vehicle model. Defaults to "iris".
            mavlink_out_host (str): Host for Pegasus mavlink (127.0.0.1 when in same container).
            mavlink_udp_port (int): Pegasus pymavlink udpin port (ONBOARD_BASE_PORT + ROS_DOMAIN_ID).
        """

        self.ardupilot_process = None
        self.vehicle_id = vehicle_id
        self.ardupilot_dir = ardupilot_dir
        self.ardupilot_model = ardupilot_model
        self.mavlink_out_host = mavlink_out_host
        self.mavlink_udp_port = mavlink_udp_port

        self.model = "JSON"

        self.root_fs = tempfile.TemporaryDirectory()
        # Build a clean env for sim_vehicle so it doesn't inherit Kit's PYTHONPATH/PYTHONHOME
        # which would make the system python3 load Kit's mismatched stdlib C-extensions.
        self.environment = {
            k: v
            for k, v in os.environ.items()
            if k not in ("PYTHONPATH", "PYTHONHOME")
        }
        # Populated in launch_ardupilot() for debugging
        self.last_command: Optional[str] = None
        self.last_stderr_log: Optional[str] = None

    def _dbg(self, msg: str) -> None:
        print(f"[ArduPilotLaunchTool] {msg}", flush=True)

    def _sim_vehicle_working_dir(self) -> str:
        """sim_vehicle uses getcwd() for instance base dir and relcurdir() for default param paths."""
        root = os.path.expanduser(self.ardupilot_dir)
        copter = os.path.join(root, "ArduCopter")
        if os.path.isdir(copter):
            return copter
        if os.path.isdir(root):
            return root
        self._dbg(
            f"WARNING: ardupilot_dir is not a valid directory ({root!r}); "
            f"using temp cwd {self.root_fs.name!r}"
        )
        return self.root_fs.name

    def _sitl_already_exists(self):
        return os.path.exists(f"{self.ardupilot_dir}/build/sitl/bin/arducopter")

    def _get_vehicle_frame(self):
        return self.ardupilot_model

    @staticmethod
    def _find_system_python() -> str:
        """Return a Python 3 interpreter that is NOT Isaac Sim's bundled one.

        Isaac's Kit bundles its own python3 under /isaac-sim/kit/python/bin/ which has
        mismatched C-extensions (_sre, etc.) when used outside Kit.  sim_vehicle.py must
        run under the *system* Python that ArduPilot was built with.
        """
        for candidate in ("/usr/bin/python3", "/usr/local/bin/python3"):
            if os.path.isfile(candidate):
                return candidate
        fallback = shutil.which("python3") or "python3"
        return fallback

    def _build_command_string(self) -> Tuple[str, Optional[str]]:
        out_port = self.mavlink_udp_port
        python = self._find_system_python()
        parts = [
            python,
            f"{self.ardupilot_dir}/Tools/autotest/sim_vehicle.py",
            "-v",
            "ArduCopter",
            "-f",
            f"{self._get_vehicle_frame()}",
            "--model",
            f"{self.model}",
        ]
        if self._sitl_already_exists():
            parts.append("--no-rebuild")

        headless = os.environ.get("ARDUPILOT_LAUNCH_HEADLESS", "").lower() in (
            "1",
            "true",
            "yes",
        )
        # No gnome-terminal (typical Docker): wx console/map will fail — treat like headless for sim_vehicle args
        if not headless and shutil.which("gnome-terminal") is None:
            headless = True
        if not headless:
            parts.extend(["--console", "--map"])

        parts.extend(
            [
                "-I",
                f"{self.vehicle_id}",
                "--sysid",
                f"{self.vehicle_id + 1}",
            ]
        )
        # sim_vehicle always tries to spawn mavproxy.py; Docker images often lack it on PATH.
        # Without MAVProxy, --out is ignored (it only applies to MAVProxy). Use --no-mavproxy and
        # send MAVLink from SITL straight to Pegasus (pymavlink udpin on this host:port).
        mavproxy = shutil.which("mavproxy.py")
        # SITL serial layout (no-mavproxy Docker path):
        #   serial0 → tcp:0.0.0.0:<5760+vehicle_id*10>:wait  (MAVROS connects here from robot container)
        #   serial1 → udpclient:127.0.0.1:<onboard_port>     (Pegasus pymavlink udpin, same container)
        serial0_port = 5760 + self.vehicle_id * 10
        sitl_args = [
            f"--serial0=tcp:0.0.0.0:{serial0_port}:wait",
            f"--serial1=udpclient:{self.mavlink_out_host}:{out_port}",
        ]
        if mavproxy:
            parts.extend(["--out", f"udp:{self.mavlink_out_host}:{out_port}"])
            # sim_vehicle.py --sitl-instance-args accepts ONE string; pass serial1 only
            parts.extend(["--sitl-instance-args", sitl_args[1]])
        else:
            parts.append("--no-mavproxy")
            # sim_vehicle.py --sitl-instance-args accepts ONE string value that gets
            # split and forwarded to the SITL binary.  Passing the flag twice causes the
            # second to silently override the first (argparse store action), so we must
            # combine all args into a single space-separated string.
            parts.extend(["--sitl-instance-args", " ".join(sitl_args)])
        # Ensure SERIAL0 (MAVROS TCP) gets position/attitude streams; some SITL defaults leave SR0_* at 0.
        stream_parm = (
            Path(__file__).resolve().parents[3]
            / "config"
            / "ardupilot_sitl_mavros_streams.parm"
        )
        if stream_parm.is_file():
            parts.extend(["--add-param-file", str(stream_parm)])
        return shlex.join(parts), mavproxy

    def launch_ardupilot(self):
        """
        Method that will launch a ardupilot instance with the specified configuration.
        Default matches legacy Pegasus: gnome-terminal with console/map unless
        ARDUPILOT_LAUNCH_HEADLESS=true (then background bash, for Docker/CI).
        ARDUPILOT_USE_GNOME_TERMINAL=true forces gnome-terminal even when headless.
        """
        command, mavproxy_path = self._build_command_string()
        self.last_command = command
        sitl_cwd = self._sim_vehicle_working_dir()
        sys_python = self._find_system_python()

        sim_vehicle_py = f"{self.ardupilot_dir}/Tools/autotest/sim_vehicle.py"
        arducopter_bin = f"{self.ardupilot_dir}/build/sitl/bin/arducopter"
        self._dbg(f"ardupilot_dir={self.ardupilot_dir!r}")
        self._dbg(f"sim_vehicle.py exists={os.path.isfile(sim_vehicle_py)} path={sim_vehicle_py}")
        self._dbg(f"arducopter binary exists={os.path.isfile(arducopter_bin)} path={arducopter_bin}")
        self._dbg(f"python for sim_vehicle: {sys_python} (avoiding Kit's bundled python)")
        stripped = [k for k in ("PYTHONPATH", "PYTHONHOME") if k in os.environ]
        if stripped:
            self._dbg(f"stripped from subprocess env: {stripped} (prevent Kit stdlib leak)")
        serial0_port = 5760 + self.vehicle_id * 10
        self._dbg(f"mavlink_udp {self.mavlink_out_host}:{self.mavlink_udp_port} vehicle_id={self.vehicle_id}")
        self._dbg(f"serial0 (MAVROS TCP) tcp:0.0.0.0:{serial0_port} — MAVROS should connect to tcp://<isaac_ip>:{serial0_port}")
        self._dbg(
            f"mavproxy.py on PATH: {mavproxy_path!r} -> "
            f"{'MAVProxy --out' if mavproxy_path else '--no-mavproxy + serial1=udpclient (no MAVProxy)'}"
        )
        self._dbg(f"bash -c command ({len(command)} chars): {command}")
        self._dbg(f"sim_vehicle subprocess cwd={sitl_cwd!r} (must be ArduPilot tree, not empty /tmp)")

        # Upstream Pegasus always used gnome-terminal + console/map. Keep that as the default
        # when ARDUPILOT_LAUNCH_HEADLESS is unset/false so behavior matches pre-fork code.
        # Headless / Docker: set ARDUPILOT_LAUNCH_HEADLESS=true to skip gnome and run sim_vehicle
        # in the background (no wx console/map).
        force_gnome = os.environ.get("ARDUPILOT_USE_GNOME_TERMINAL", "").lower() in (
            "1",
            "true",
            "yes",
        )
        headless = os.environ.get("ARDUPILOT_LAUNCH_HEADLESS", "").lower() in (
            "1",
            "true",
            "yes",
        )
        gnome = shutil.which("gnome-terminal")
        # Prefer gnome only when it exists and we want a GUI shell (not headless, or forced)
        use_gnome = gnome and ((not headless) or force_gnome)

        # Capture sim_vehicle stdout+stderr to a log file so we can diagnose failures.
        # sim_vehicle prints errors with print() (stdout) and Python tracebacks go to stderr.
        log_dest = None
        self.last_stderr_log = None
        if use_gnome:
            self._dbg(
                "output: gnome-terminal path — sim_vehicle output goes to the terminal window "
                "(no log file)"
            )
        else:
            log_env = os.environ.get("ARDUPILOT_SIM_VEHICLE_LOG", "").strip()
            if log_env.lower() in ("0", "false", "no", "none"):
                self._dbg("output: discarded (ARDUPILOT_SIM_VEHICLE_LOG=0|false|no|none)")
            elif log_env:
                self.last_stderr_log = os.path.abspath(log_env)
                log_dest = open(
                    self.last_stderr_log, "a", encoding="utf-8", errors="replace"
                )
                self._dbg(
                    f"stdout+stderr: appending to ARDUPILOT_SIM_VEHICLE_LOG={self.last_stderr_log!r}"
                )
            else:
                self.last_stderr_log = os.path.join(
                    tempfile.gettempdir(),
                    f"pegasus_sim_vehicle_{self.vehicle_id}.log",
                )
                log_dest = open(
                    self.last_stderr_log, "w", encoding="utf-8", errors="replace"
                )
                self._dbg(f"stdout+stderr: writing to {self.last_stderr_log!r}")

        popen_kw = dict(
            cwd=sitl_cwd,
            shell=False,
            env=self.environment,
            preexec_fn=os.setsid,
            stdout=log_dest if self.last_stderr_log else subprocess.DEVNULL,
            stderr=log_dest if self.last_stderr_log else subprocess.DEVNULL,
        )

        if use_gnome:
            self._dbg(f"launch via gnome-terminal: {gnome!r}")
            self.ardupilot_process = subprocess.Popen(
                [gnome, "--", "bash", "-c", command],
                **popen_kw,
            )
        else:
            if not gnome and not headless and not force_gnome:
                self._dbg(
                    "gnome-terminal not in PATH; running sim_vehicle via bash "
                    "(same as ARDUPILOT_LAUNCH_HEADLESS=true)"
                )
            self.ardupilot_process = subprocess.Popen(
                ["bash", "-c", command],
                **popen_kw,
            )

        pid = self.ardupilot_process.pid
        wrapper = "gnome-terminal" if use_gnome else "bash"
        self._dbg(f"Popen returned {wrapper} pid={pid} cwd={sitl_cwd!r}")
        time.sleep(0.35)
        rc = self.ardupilot_process.poll()
        if rc is not None:
            self._dbg(f"WARNING: {wrapper} wrapper exited immediately with code={rc}")
            if self.last_stderr_log and log_dest is not None:
                log_dest.flush()
                try:
                    with open(self.last_stderr_log, "r", encoding="utf-8", errors="replace") as f:
                        contents = f.read(8192)
                    if contents.strip():
                        self._dbg(f"--- sim_vehicle output ({self.last_stderr_log}) ---")
                        for line in contents.splitlines()[:60]:
                            self._dbg(f"  | {line}")
                        self._dbg("--- end sim_vehicle output ---")
                    else:
                        self._dbg(f"sim_vehicle log is empty: {self.last_stderr_log}")
                except Exception as e:
                    self._dbg(f"Could not read log file: {e!r}")
        else:
            self._dbg(
                f"{wrapper} wrapper still running after 0.35s (child may still be starting)"
            )

        # Best-effort: show matching lines from ps (arducopter / mavproxy / sim_vehicle)
        try:
            ps_out = subprocess.run(
                ["ps", "-eo", "pid,args"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if ps_out.returncode == 0:
                hits = [
                    ln
                    for ln in ps_out.stdout.splitlines()
                    if any(
                        k in ln
                        for k in (
                            "arducopter",
                            "mavproxy",
                            "sim_vehicle.py",
                            str(pid),
                        )
                    )
                ]
                self._dbg(f"ps filter (pid/arducopter/mavproxy/sim_vehicle): {len(hits)} line(s)")
                for ln in hits[:12]:
                    self._dbg(f"  {ln.strip()}")
                if len(hits) > 12:
                    self._dbg(f"  ... ({len(hits) - 12} more)")
        except Exception as e:
            self._dbg(f"ps snapshot failed: {e!r}")

    def kill_ardupilot(self):
        """
        Method that will kill a ardupilot instance with the specified configuration
        """
        if self.ardupilot_process is not None:
            try:
                os.killpg(self.ardupilot_process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.ardupilot_process.kill()
                self.ardupilot_process.wait(timeout=5)
            except Exception:
                pass
            self.ardupilot_process = None

        keywords = ["arducopter", "mavproxy"]

        ps_output = subprocess.run(["ps", "-aux"], capture_output=True, text=True)

        matching_processes = [
            line
            for line in ps_output.stdout.splitlines()
            if any(keyword in line.lower() for keyword in keywords)
        ]

        pids = [line.split()[1] for line in matching_processes]

        for pid in pids:
            try:
                os.kill(int(pid), 9)
                print(f"Killed process {pid}")
            except ProcessLookupError:
                print(f"Process {pid} not found.")
            except PermissionError:
                print(f"Permission denied to kill process {pid}.")
            except Exception as e:
                print(f"Failed to kill process {pid}: {e}")

    def __del__(self):
        """
        If the ardupilot process is still running when the Ardupilot launch tool object is whiped from memory, then make sure
        we kill the ardupilot instance so we don't end up with hanged ardupilot instances
        """

        if self.ardupilot_process:
            self.kill_ardupilot()

        self.root_fs.cleanup()


def main():
    ardupilot_tool = ArduPilotLaunchTool(os.environ["HOME"] + "/ardupilot")
    print("Launching ArduPilot")
    ardupilot_tool.launch_ardupilot()

    import time

    time.sleep(3000)


if __name__ == "__main__":
    main()
