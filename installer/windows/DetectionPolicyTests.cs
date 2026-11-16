using System;
using System.Linq;
using System.IO;
public static class DetectionPolicyTests {
    static void Check(bool pass, string message) { if(!pass) throw new Exception(message); }
    public static void Main() {
        var devices=new[] {
            new PortDescriptor{Port="COM2",Name="USB Serial Device (COM2)",HardwareId=@"USB\VID_303A&PID_1001\ABC"},
            new PortDescriptor{Port="COM2",Name="M5StickS3 (COM2)",HardwareId=""},
            new PortDescriptor{Port="COM9",Name="USB Serial Device (COM9)",HardwareId=@"USB\VID_303A&PID_1001&MI_00\DEF"},
            new PortDescriptor{Port="COM12",Name="Arduino (COM12)",HardwareId=@"USB\VID_2341&PID_0043\X"},
            new PortDescriptor{Port="COM13",Name="USB (COM13)",HardwareId=@"USB\VID_303A&PID_10010\X"},
            new PortDescriptor{Port="COM99",Name="M5Stack StickS3 (COM99)",HardwareId=""}
        };
        var found=DetectionPolicy.Candidates(devices,new[]{"COM2","COM9","COM12","COM13"});
        Check(found.Count==2,"filter/deduplicate only supported active ports");
        Check(found[0].HasBoardName,"prefer specific name for duplicate COM");
        Check(DetectionPolicy.UniquePort(found)==null,"ambiguous ports must not auto select");
        Check(DetectionPolicy.UniquePort(found.Take(1))=="COM2","one verified port");
        Check(DetectionPolicy.UniquePort(new PortDescriptor[0])==null,"not found");
        Check(DetectionPolicy.IsEsp32S3(0,"Connected to ESP32-S3 on COM2:\nChip type: ESP32-S3 (QFN56)"),"v5 chip output");
        Check(DetectionPolicy.IsEsp32S3(0,"Chip is ESP32-S3 (revision v0.2)"),"legacy chip output");
        Check(!DetectionPolicy.IsEsp32S3(2,"Chip type: ESP32-S3"),"failed probe cannot pass");
        Check(!DetectionPolicy.IsEsp32S3(0,"Chip type: ESP32-S2\nMAC: 11:22"),"wrong chip");
        Check(!DetectionPolicy.IsEsp32S3(0,"ESP32-S3 was requested but connection failed"),"error text not confirmation");
        foreach(string port in new[]{"COM0","COM999999999999999","COM2 --erase-flash","../COM3","COM65536"})
            Check(!PortDescriptor.IsValidPort(port),"invalid port: "+port);
        string args=DetectionPolicy.ProbeArguments("COM2");
        Check(args.Contains("read-mac") && args.Contains("--no-stub") && !args.Contains("write") && !args.Contains("erase"),"read-only probe command");
        // Exercise the real production Package parser from Installer.cs.
        string root=Path.Combine(Path.GetTempPath(),"pano-installer-tests-"+Guid.NewGuid());
        Directory.CreateDirectory(Path.Combine(root,"firmware"));
        File.WriteAllBytes(Path.Combine(root,"firmware","app.bin"),new byte[8192]);
        Action<string,bool> validate=(config,valid)=>{
            File.WriteAllText(Path.Combine(root,"firmware.txt"),config);
            bool success;
            try { Package.Read(root).Command("COM2"); success=true; } catch { success=false; }
            Check(success==valid,"package case "+config);
        };
        validate("name=Pano-Checker\nchip=esp32s3\nmode=dio\nfreq=80m\nsize=8MB\n0x0=app.bin",true);
        validate("chip=esp32s3\n0x0=missing.bin",false);
        validate("chip=esp32s3\n0x0=app.bin\n0x1000=app.bin",false);
        validate("chip=esp32s3\n0x1=app.bin",false);
        validate("chip=esp32s3\n0x0=../app.bin",false);
        validate("chip=esp32s3",false);
        Directory.Delete(root,true);
        Console.WriteLine("PASS: USB candidates, duplicates, ambiguity, no match, probe output/exit status, argument safety, real package parser");
    }
}
