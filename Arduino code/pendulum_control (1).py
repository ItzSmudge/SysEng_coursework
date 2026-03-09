#!/usr/bin/env python3
"""
Arduino Giga Inverted Pendulum WiFi Control Script
Provides WiFi-based control for START, STOP, RESET_ENCODER, and gain adjustment
"""

import socket
import json
import sys
import time
from typing import Optional, Dict, Any
import cmd


class GigaPendulumController:
    """Control class for Arduino Giga pendulum via WiFi"""

    def __init__(self, host: str, port: int = 8080, timeout: float = 5.0):
        """
        Initialize controller

        Args:
            host: IP address of Arduino Giga
            port: WiFi server port (default 8080)
            timeout: Socket timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False

    def connect(self) -> bool:
        """Establish connection to Arduino Giga"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"✓ Connected to Giga at {self.host}:{self.port}")
            return True
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            print(f"✗ Connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        """Close connection"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False
        print("Disconnected")

    def send_command(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Send command to Arduino and receive JSON response

        Args:
            command: Command string to send

        Returns:
            Parsed JSON response or None if error
        """
        if not self.connected:
            print("Not connected to Giga. Use 'connect' command first.")
            return None

        try:
            # Send command
            self.socket.sendall((command + '\n').encode('utf-8'))

            # Receive response
            response_data = b''
            while True:
                chunk = self.socket.recv(1024)
                if not chunk:
                    break
                response_data += chunk
                # Check if we have a complete JSON response (ends with \n)
                if b'\n' in response_data:
                    break

            response_str = response_data.decode('utf-8').strip()

            # Parse JSON response
            if response_str:
                response = json.loads(response_str)
                return response
            else:
                print("Empty response from Giga")
                return None

        except socket.timeout:
            print("✗ Command timeout")
            self.connected = False
            return None
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON response: {e}")
            print(f"   Raw response: {response_str}")
            return None
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            self.connected = False
            return None

    def start(self) -> bool:
        """Start the pendulum controller"""
        response = self.send_command("START")
        if response and response.get("status") == "ok":
            print("✓ System started")
            return True
        else:
            print("✗ Failed to start system")
            return False

    def stop(self) -> bool:
        """Stop the pendulum controller"""
        response = self.send_command("STOP")
        if response and response.get("status") == "ok":
            print("✓ System stopped")
            return True
        else:
            print("✗ Failed to stop system")
            return False

    def reset_encoder(self) -> bool:
        """Reset encoder count to zero"""
        response = self.send_command("RESET_ENCODER")
        if response and response.get("status") == "ok":
            print("✓ Encoder reset to 0")
            return True
        else:
            print("✗ Failed to reset encoder")
            return False

    def set_gains(self, kp_t: float, kd_t: float, ki_t: float,
                  kp_x: float, kd_x: float, ki_x: float) -> bool:
        """
        Set PID gains

        Args:
            kp_t: Theta proportional gain
            kd_t: Theta derivative gain
            ki_t: Theta integral gain
            kp_x: Position proportional gain
            kd_x: Position derivative gain
            ki_x: Position integral gain
        """
        command = f"SET_GAINS {kp_t} {kd_t} {ki_t} {kp_x} {kd_x} {ki_x}"
        response = self.send_command(command)

        if response and response.get("status") == "ok":
            print("✓ PID gains updated:")
            print(f"  Theta: Kp={kp_t}, Kd={kd_t}, Ki={ki_t}")
            print(f"  Position: Kp={kp_x}, Kd={kd_x}, Ki={ki_x}")
            return True
        else:
            print("✗ Failed to update gains")
            return False

    def get_status(self) -> Optional[Dict[str, Any]]:
        """Get system status"""
        response = self.send_command("STATUS")
        if response and response.get("status") == "ok":
            return response
        else:
            print("✗ Failed to get status")
            return None

    def print_status(self) -> None:
        """Print formatted system status"""
        status = self.get_status()
        if status:
            print("\n" + "=" * 50)
            print("SYSTEM STATUS")
            print("=" * 50)
            running = "RUNNING" if status.get("system_running") else "STOPPED"
            enabled = "ENABLED" if status.get("system_enabled") else "DISABLED"
            print(f"Status: {running} ({enabled})")
            print(f"Theta: {status.get('theta_rad', 0):.4f} rad ({status.get('theta_deg', 0):.2f}°)")
            print(f"Theta rate: {status.get('theta_dot', 0):.4f} rad/s")
            print(f"Encoder count: {status.get('encoder_count', 0)}")
            print(f"Integral theta: {status.get('int_theta', 0):.4f}")
            print(f"Integral x: {status.get('int_x', 0):.4f}")
            print("=" * 50 + "\n")


class PendulumCLI(cmd.Cmd):
    """Interactive CLI for pendulum control"""

    intro = """
╔════════════════════════════════════════════════════╗
║   Arduino Giga Inverted Pendulum WiFi Control     ║
╚════════════════════════════════════════════════════╝

Type 'help' for available commands
    """
    prompt = "giga> "

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controller: Optional[GigaPendulumController] = None

    def do_connect(self, arg):
        """
        Connect to Arduino Giga
        Usage: connect <IP_ADDRESS> [port]
        Example: connect 192.168.1.100
        Example: connect 192.168.1.100 8080
        """
        args = arg.split()
        if not args:
            print("Usage: connect <IP_ADDRESS> [port]")
            return

        host = args[0]
        port = int(args[1]) if len(args) > 1 else 8080

        self.controller = GigaPendulumController(host, port)
        if self.controller.connect():
            self.prompt = f"giga({host})> "
        else:
            self.controller = None

    def do_disconnect(self, arg):
        """Disconnect from Arduino Giga"""
        if self.controller:
            self.controller.disconnect()
            self.controller = None
            self.prompt = "giga> "

    def do_start(self, arg):
        """Start the pendulum controller"""
        if self._check_connection():
            self.controller.start()

    def do_stop(self, arg):
        """Stop the pendulum controller"""
        if self._check_connection():
            self.controller.stop()

    def do_reset_encoder(self, arg):
        """Reset the encoder count to zero"""
        if self._check_connection():
            self.controller.reset_encoder()

    def do_status(self, arg):
        """Get and display system status"""
        if self._check_connection():
            self.controller.print_status()

    def do_set_gains(self, arg):
        """
        Set PID controller gains
        Usage: set_gains <kp_theta> <kd_theta> <ki_theta> <kp_x> <kd_x> <ki_x>
        Example: set_gains 10000 3000 300 30 15 0.3
        """
        args = arg.split()
        if len(args) < 6:
            print("Usage: set_gains <kp_theta> <kd_theta> <ki_theta> <kp_x> <kd_x> <ki_x>")
            return

        try:
            gains = [float(x) for x in args[:6]]
            if self._check_connection():
                self.controller.set_gains(*gains)
        except ValueError:
            print("Error: All gains must be numbers")

    def do_monitor(self, arg):
        """
        Monitor system status in real-time
        Usage: monitor [interval]
        Example: monitor 0.5 (updates every 0.5 seconds)
        """
        if not self._check_connection():
            return

        interval = 1.0
        if arg:
            try:
                interval = float(arg)
            except ValueError:
                print("Invalid interval")
                return

        print("Monitoring (Ctrl+C to stop)...\n")
        try:
            while True:
                status = self.controller.get_status()
                if status and status.get("status") == "ok":
                    running = "●" if status.get("system_running") else "○"
                    theta = status.get('theta_deg', 0)
                    theta_dot = status.get('theta_dot', 0)
                    print(
                        f"{running} θ={theta:7.2f}° | θ̇={theta_dot:8.4f} rad/s | "
                        f"count={status.get('encoder_count', 0):6d}"
                    )
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped")

    def do_quick_tune(self, arg):
        """
        Interactive quick tuning wizard for PID gains
        Usage: quick_tune
        """
        if not self._check_connection():
            return

        print("\n" + "=" * 50)
        print("PID GAIN QUICK TUNE WIZARD")
        print("=" * 50)

        try:
            kp_t = float(input("Theta Kp [10000]: ") or "10000")
            kd_t = float(input("Theta Kd [3000]: ") or "3000")
            ki_t = float(input("Theta Ki [300]: ") or "300")
            kp_x = float(input("Position Kp [30]: ") or "30")
            kd_x = float(input("Position Kd [15]: ") or "15")
            ki_x = float(input("Position Ki [0.3]: ") or "0.3")

            print(f"\nApplying gains...")
            self.controller.set_gains(kp_t, kd_t, ki_t, kp_x, kd_x, ki_x)
            print("=" * 50 + "\n")

        except ValueError:
            print("Error: Invalid input")
        except KeyboardInterrupt:
            print("\nCancelled")

    def do_preset(self, arg):
        """
        Load preset gain configurations
        Usage: preset <name>
        Available presets: aggressive, balanced, conservative
        """
        presets = {
            "aggressive": (15000, 4000, 500, 40, 20, 0.5),
            "balanced": (10000, 3000, 300, 30, 15, 0.3),
            "conservative": (5000, 2000, 150, 20, 10, 0.1),
        }

        if not arg or arg not in presets:
            print("Available presets:", ", ".join(presets.keys()))
            return

        gains = presets[arg]
        if self._check_connection():
            self.controller.set_gains(*gains)
            print(f"✓ Loaded '{arg}' preset")

    def do_help(self, arg):
        """Show help information"""
        if not arg:
            print(self.__doc__)
            print("\nAvailable commands:")
            print("  connect <ip> [port]  - Connect to Arduino Giga")
            print("  disconnect           - Disconnect from Giga")
            print("  start                - Start the system")
            print("  stop                 - Stop the system")
            print("  reset_encoder        - Reset encoder to zero")
            print("  status               - Get current system status")
            print("  set_gains <vals>     - Set PID gains (6 values)")
            print("  monitor [interval]   - Real-time status monitoring")
            print("  quick_tune           - Interactive tuning wizard")
            print("  preset <name>        - Load preset gains")
            print("  help [cmd]           - Show help for command")
            print("  exit                 - Exit program")
        else:
            super().do_help(arg)

    def do_exit(self, arg):
        """Exit the program"""
        if self.controller:
            self.controller.disconnect()
        print("Goodbye!")
        return True

    def do_EOF(self, arg):
        """Handle Ctrl+D"""
        print()
        return self.do_exit(arg)

    def emptyline(self):
        """Do nothing on empty input"""
        pass

    def _check_connection(self) -> bool:
        """Check if connected to controller"""
        if not self.controller or not self.controller.connected:
            print("Not connected. Use 'connect <IP>' first")
            return False
        return True


def main():
    """Main entry point"""
    if len(sys.argv) > 1:
        # Command line mode
        host = sys.argv[1]
        port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

        controller = GigaPendulumController(host, port)
        if not controller.connect():
            sys.exit(1)

        # Process commands from stdin or arguments
        if len(sys.argv) > 3:
            # Direct command
            command = " ".join(sys.argv[3:])
            response = controller.send_command(command)
            if response:
                print(json.dumps(response, indent=2))
        else:
            # Interactive mode
            cli = PendulumCLI()
            cli.controller = controller
            cli.prompt = f"giga({host})> "
            cli.cmdloop()

        controller.disconnect()
    else:
        # Interactive mode only
        cli = PendulumCLI()
        cli.cmdloop()


if __name__ == "__main__":
    main()
