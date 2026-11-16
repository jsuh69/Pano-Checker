using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;

public sealed class PortDescriptor {
    public string Port, Name, HardwareId;
    public bool HasBoardName {
        get { return Regex.IsMatch(Name ?? "", @"\b(?:M5\s*)?Stick\s*S3\b", RegexOptions.IgnoreCase); }
    }
    public bool IsCandidate {
        get { return IsValidPort(Port) && (HasBoardName ||
            Regex.IsMatch(HardwareId ?? "", @"VID_303A&PID_1001(?:&|\\|$)", RegexOptions.IgnoreCase)); }
    }
    public static bool IsValidPort(string port) {
        int number;
        return Regex.IsMatch(port ?? "", @"\ACOM[0-9]+\z", RegexOptions.IgnoreCase)
            && int.TryParse(port.Substring(3), out number) && number > 0 && number <= 65535;
    }
}
public static class DetectionPolicy {
    public static List<PortDescriptor> Candidates(IEnumerable<PortDescriptor> devices, IEnumerable<string> activePorts) {
        var active = new HashSet<string>(activePorts, StringComparer.OrdinalIgnoreCase);
        return devices.Where(d => d.IsCandidate && active.Contains(d.Port))
            .GroupBy(d => d.Port, StringComparer.OrdinalIgnoreCase)
            .Select(g => g.OrderByDescending(d => d.HasBoardName).First())
            .OrderBy(d => int.Parse(d.Port.Substring(3))).ToList();
    }
    public static bool IsEsp32S3(int exitCode, string output) {
        return exitCode == 0 && Regex.IsMatch(output ?? "",
            @"(?im)^\s*(?:Chip type:\s*|Chip is\s+|Connected to\s+|Detecting chip type\.\.\.\s*)ESP32-S3\b");
    }
    public static string ProbeArguments(string port) {
        if (!PortDescriptor.IsValidPort(port)) throw new ArgumentException("Invalid COM port");
        return "--chip auto --port " + port + " --baud 115200 --before default-reset --after hard-reset"
            + " --connect-attempts 2 --no-stub read-mac";
    }
    public static string UniquePort(IEnumerable<PortDescriptor> verified) {
        var ports = verified.Select(d => d.Port).Distinct(StringComparer.OrdinalIgnoreCase).ToArray();
        return ports.Length == 1 ? ports[0] : null;
    }
}
