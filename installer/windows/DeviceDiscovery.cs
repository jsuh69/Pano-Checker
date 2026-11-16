using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Management;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

public static class DeviceDiscovery {
    public static List<PortDescriptor> Find(string root, Action<string> log) {
        var devices = new List<PortDescriptor>();
        var options = new EnumerationOptions { Timeout = TimeSpan.FromSeconds(5), ReturnImmediately = false };
        using (var search = new ManagementObjectSearcher(new ManagementScope(@"\\.\root\CIMV2"),
            new ObjectQuery("SELECT Name, PNPDeviceID FROM Win32_PnPEntity WHERE Name LIKE '%(COM%'"), options))
        using (var collection = search.Get()) {
            foreach (ManagementObject item in collection) using (item) {
                var name = Convert.ToString(item["Name"]);
                var match = Regex.Match(name, @"\((COM[0-9]+)\)", RegexOptions.IgnoreCase);
                if (match.Success) devices.Add(new PortDescriptor {
                    Port = match.Groups[1].Value.ToUpperInvariant(), Name = name,
                    HardwareId = Convert.ToString(item["PNPDeviceID"])
                });
            }
        }
        var candidates = DetectionPolicy.Candidates(devices, SerialPort.GetPortNames());
        log("연결된 M5StickS3/ESP32-S3 후보: " + candidates.Count + "개");
        var verified = new List<PortDescriptor>();
        foreach (var device in candidates) {
            log(device.Port + " 확인 중: " + device.Name);
            try {
                string output;
                int code = Probe(Path.Combine(root, "tools", "esptool.exe"), root, device.Port, out output);
                log(output);
                if (DetectionPolicy.IsEsp32S3(code, output)) verified.Add(device);
                else log(device.Port + ": ESP32-S3 확인 실패 (다른 칩, 연결 실패 또는 시간 초과)");
            } catch (Exception e) { log(device.Port + ": " + e.Message); }
        }
        return verified;
    }
    static int Probe(string executable, string root, string port, out string output) {
        using (var proc = new Process()) {
            proc.StartInfo = new ProcessStartInfo(executable, DetectionPolicy.ProbeArguments(port)) {
                WorkingDirectory = root, UseShellExecute = false, CreateNoWindow = true,
                RedirectStandardOutput = true, RedirectStandardError = true
            };
            proc.Start();
            Task<string> stdout = proc.StandardOutput.ReadToEndAsync();
            Task<string> stderr = proc.StandardError.ReadToEndAsync();
            bool exited = proc.WaitForExit(12000);
            if (!exited) {
                // esptool's one-file launcher can have a child process. Kill only
                // this probe's process tree so a timed-out scan cannot retain COM.
                try {
                    using (var kill = Process.Start(new ProcessStartInfo("taskkill.exe",
                        "/PID " + proc.Id + " /T /F") { UseShellExecute = false, CreateNoWindow = true }))
                        kill.WaitForExit(3000);
                } catch { try { proc.Kill(); } catch { } }
                proc.WaitForExit(2000);
            }
            try { Task.WaitAll(new Task[] { stdout, stderr }, 2000); } catch { }
            output = (stdout.Status == TaskStatus.RanToCompletion ? stdout.Result : "") + "\n"
                + (stderr.Status == TaskStatus.RanToCompletion ? stderr.Result : "");
            if (!exited) output += "\n시간 초과: " + port;
            return exited ? proc.ExitCode : -1;
        }
    }
}
