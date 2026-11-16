using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using System.Globalization;

class Package {
    public string Chip, Name, Version, Args;
    public static Package Read(string root) {
        var config = new Dictionary<string,string>(StringComparer.Ordinal);
        var images = new SortedDictionary<long,long>();
        var cmd = new StringBuilder();
        foreach (string raw in File.ReadAllLines(Path.Combine(root,"firmware.txt"))) {
            string line=raw.Trim();
            if(line.Length==0 || line.StartsWith("#")) continue;
            int split=line.IndexOf('=');
            if(split<1) throw new FormatException("firmware.txt: " + line);
            string key=line.Substring(0,split).Trim(), value=line.Substring(split+1).Trim();
            if(!key.StartsWith("0x")) {
                if(!new[]{"name","version","chip","mode","freq","size"}.Contains(key) || config.ContainsKey(key))
                    throw new FormatException("설정 키 확인: " + key);
                config.Add(key,value);
                continue;
            }
            long addr;
            if(!Regex.IsMatch(key,@"\A0x[0-9a-fA-F]{1,8}\z") ||
               !long.TryParse(key.Substring(2),NumberStyles.HexNumber,CultureInfo.InvariantCulture,out addr) ||
               addr % 4096 != 0 || !Regex.IsMatch(value,@"\A[A-Za-z0-9_-][A-Za-z0-9_.-]*\.bin\z",RegexOptions.IgnoreCase))
                throw new FormatException("플래시 주소 또는 파일 이름 확인: " + line);
            var file=new FileInfo(Path.Combine(root,"firmware",value));
            if(!file.Exists) throw new FileNotFoundException("펌웨어 파일 없음: " + value);
            long end=addr+file.Length;
            if(file.Length==0 || end>0x100000000L || images.Any(p=>addr<p.Value && p.Key<end))
                throw new FormatException("파일이 비어 있거나 쓰기 영역이 겹칩니다: " + line);
            images.Add(addr,end);
            cmd.AppendFormat(" 0x{0:x} firmware/{1}",addr,value);
        }
        string chip=Get(config,"chip","");
        if(!Regex.IsMatch(chip,@"\Aesp32(?:[a-z][a-z0-9]*)?\z") || images.Count==0)
            throw new FormatException("대상 ESP32 칩과 펌웨어 주소를 설정하세요.");
        string mode=Get(config,"mode","keep"), freq=Get(config,"freq","keep"), size=Get(config,"size","keep");
        if(!Regex.IsMatch(mode,@"\A(keep|qio|qout|dio|dout)\z") ||
           !Regex.IsMatch(freq,@"\A(keep|[0-9]{1,3}m)\z") ||
           !Regex.IsMatch(size,@"\A(keep|detect|[0-9]{1,3}MB)\z"))
            throw new FormatException("mode, freq, size 값을 확인하세요.");
        return new Package{Chip=chip, Name=Get(config,"name","ESP32"),Version=Get(config,"version",""),
            Args="write-flash --flash-mode "+mode+" --flash-freq "+freq+" --flash-size "+size+cmd};
    }
    static string Get(Dictionary<string,string> d,string k,string fallback) { string v; return d.TryGetValue(k,out v)?v:fallback; }
    public string Command(string port) {
        if(!Regex.IsMatch(port,@"\ACOM[0-9]+\z",RegexOptions.IgnoreCase)) throw new FormatException("COM 포트 확인");
        return "--chip "+Chip+" --port "+port+" --baud 115200 --before default-reset --after hard-reset "+Args;
    }
}
class Installer : Form {
    readonly string root=AppDomain.CurrentDomain.BaseDirectory;
    readonly ComboBox ports=new ComboBox();
    readonly Button refresh=new Button(), find=new Button(), install=new Button();
    readonly Label title=new Label(), status=new Label();
    readonly TextBox log=new TextBox();
    Package package;
    bool busy;
    public Installer() {
        Text="ESP32 펌웨어 설치";
        ClientSize=new Size(700,425); MinimumSize=new Size(650,430);
        StartPosition=FormStartPosition.CenterScreen; Font=new Font("Malgun Gothic",10);
        title.SetBounds(20,16,650,36); title.Font=new Font(Font.FontFamily,16,FontStyle.Bold);
        title.AutoEllipsis=true; title.Text=Text;
        var guide=new Label{Text="COM 포트를 선택하거나 M5StickS3 찾기를 누르세요. 검색 시 후보 보드가 재시작될 수 있습니다."};
        guide.SetBounds(20,59,650,30);
        ports.SetBounds(20,98,155,32); ports.DropDownStyle=ComboBoxStyle.DropDownList;
        find.SetBounds(188,96,156,33); find.Text="M5StickS3 찾기";
        refresh.SetBounds(357,96,104,33); refresh.Text="새로 고침";
        install.SetBounds(474,96,104,33); install.Text="설치";
        status.SetBounds(20,143,660,48); status.Anchor=AnchorStyles.Left|AnchorStyles.Right|AnchorStyles.Top;
        log.SetBounds(20,196,660,207); log.Multiline=true; log.ReadOnly=true;
        log.ScrollBars=ScrollBars.Vertical; log.Anchor=AnchorStyles.Top|AnchorStyles.Bottom|AnchorStyles.Left|AnchorStyles.Right;
        log.Font=new Font("Consolas",10); log.BackColor=Color.White;
        Controls.AddRange(new Control[]{title,guide,ports,find,refresh,install,status,log});
        refresh.Click+=(s,e)=>RefreshPorts();
        ports.SelectedIndexChanged+=(s,e)=>UpdateButton();
        install.Click+=Install;
        find.Click+=FindBoard;
        FormClosing+=(s,e)=>{if(busy){e.Cancel=true; System.Media.SystemSounds.Beep.Play();}};
        Shown+=(s,e)=>RefreshPorts();
    }
    void UpdateButton(){
        install.Enabled=!busy && package!=null && ports.SelectedIndex>=0;
        find.Enabled=!busy && File.Exists(Path.Combine(root,"tools","esptool.exe"));
    }
    void RefreshPorts(){
        package=null;
        try {
            package=Package.Read(root);
            if(!File.Exists(Path.Combine(root,"tools","esptool.exe"))) {
                package=null; throw new FileNotFoundException("tools 폴더에 공식 esptool.exe가 없습니다.");
            }
            title.Text=(package.Name+" "+package.Version+"  ·  "+package.Chip).Trim();
            var old=ports.SelectedItem as string;
            ports.Items.Clear();
            ports.Items.AddRange(SerialPort.GetPortNames().Where(p=>Regex.IsMatch(p,@"\ACOM[0-9]+\z",RegexOptions.IgnoreCase)).Distinct(StringComparer.OrdinalIgnoreCase).OrderBy(p=>p).Cast<object>().ToArray());
            if(old!=null && ports.Items.Contains(old)) ports.SelectedItem=old;
            else if(ports.Items.Count==1) ports.SelectedIndex=0;
            status.Text=ports.Items.Count==0?"COM 포트가 없습니다. USB 케이블과 보드 드라이버를 확인하세요.":"보드의 COM 포트를 확인한 뒤 설치를 누르세요.";
        } catch(Exception e){package=null; status.Text="설치 준비 필요"; Append(e.Message);}
        UpdateButton();
    }
    async void FindBoard(object sender,EventArgs e) {
        if(busy) return;
        busy=true; UpdateButton(); refresh.Enabled=ports.Enabled=false;
        status.Text="M5StickS3 연결 후보를 확인 중입니다. 후보 보드가 재시작될 수 있습니다.";
        Append("--- M5StickS3 찾기 (펌웨어 쓰기 없음) ---");
        string notice=null;
        try {
            var found=await System.Threading.Tasks.Task.Run(()=>DeviceDiscovery.Find(root,Post));
            // USB reset or removal can change the list while scanning.
            var active=SerialPort.GetPortNames();
            found=found.Where(d=>active.Contains(d.Port,StringComparer.OrdinalIgnoreCase)).ToList();
            var port=DetectionPolicy.UniquePort(found);
            var old=ports.SelectedItem as string;
            ports.Items.Clear();
            ports.Items.AddRange(active.Where(PortDescriptor.IsValidPort)
                .Distinct(StringComparer.OrdinalIgnoreCase).OrderBy(p=>p).Cast<object>().ToArray());
            if(port!=null) {
                ports.SelectedItem=port;
                status.Text="M5StickS3 연결 후보: "+port+" (ESP32-S3 확인). 기기를 확인하고 설치를 누르세요.";
                Append("자동 선택: "+port+". 같은 USB 식별자/칩을 쓰는 다른 보드와는 완전히 구별할 수 없습니다.");
            } else {
                if(old!=null && ports.Items.Contains(old)) ports.SelectedItem=old;
                notice=found.Count==0 ? "M5StickS3를 찾지 못했습니다. 데이터 케이블, 전원, 포트 사용 여부를 확인하거나 COM 포트를 수동으로 선택하세요."
                    : "여러 ESP32-S3 연결 후보를 찾았습니다: "+String.Join(", ",found.Select(d=>d.Port))
                    +". M5StickS3만 연결하거나 COM 포트를 수동으로 선택하세요.";
                status.Text=notice; Append(notice);
            }
        } catch(Exception ex) {
            notice="M5StickS3를 찾지 못했습니다. 포트 정보를 읽을 수 없습니다. 수동 선택을 사용하세요.";
            status.Text=notice; Append(ex.Message);
        } finally { busy=false;refresh.Enabled=ports.Enabled=true;UpdateButton(); }
        if(notice!=null) MessageBox.Show(this,notice,"M5StickS3 찾기",MessageBoxButtons.OK,MessageBoxIcon.Information);
    }
    void Append(string line) {
        if(line==null) return;
        if(log.TextLength>100000) log.Clear();
        log.AppendText(line+Environment.NewLine);
    }
    async void Install(object sender,EventArgs e){
        string port=ports.SelectedItem as string;
        if(busy || package==null || port==null) return;
        try {
            var current=Package.Read(root);
            string args=current.Command(port);
            if(MessageBox.Show(this,port+"에 "+current.Name+" "+current.Version+" ("+current.Chip+") 펌웨어를 설치합니다. 기존 펌웨어와 이미지가 덮는 영역의 데이터가 바뀝니다. 계속할까요?",
                Text,MessageBoxButtons.OKCancel,MessageBoxIcon.Warning,MessageBoxDefaultButton.Button2)!=DialogResult.OK) return;
            busy=true; UpdateButton(); refresh.Enabled=ports.Enabled=false;
            log.Clear(); status.Text="설치 중입니다. USB 케이블을 분리하지 마세요.";
            using(var proc=new Process()){
                proc.StartInfo=new ProcessStartInfo(Path.Combine(root,"tools","esptool.exe"),args){
                    WorkingDirectory=root,UseShellExecute=false,CreateNoWindow=true,
                    RedirectStandardOutput=true,RedirectStandardError=true};
                proc.OutputDataReceived+=(s,a)=>Post(a.Data);
                proc.ErrorDataReceived+=(s,a)=>Post(a.Data);
                proc.Start(); proc.BeginOutputReadLine(); proc.BeginErrorReadLine();
                int code=await System.Threading.Tasks.Task.Run(()=>{proc.WaitForExit();return proc.ExitCode;});
                status.Text=code==0?"설치가 완료되었습니다. 보드가 다시 시작됩니다.":"설치 실패 ("+code+"). 아래 로그를 확인하세요.";
                if(code!=0) Append("연결 오류라면 시리얼 모니터를 닫고 다시 시도하세요. 필요하면 보드의 BOOT/RESET 절차를 따르세요.");
            }
        } catch(Exception ex){status.Text="설치를 완료하지 못했습니다.";Append(ex.Message);}
        finally{busy=false;refresh.Enabled=ports.Enabled=true;UpdateButton();}
    }
    void Post(string line){
        if(line!=null && !IsDisposed && IsHandleCreated)
            BeginInvoke((Action)(()=>{if(!IsDisposed) Append(line);}));
    }
    [STAThread] static int Main(string[] args){
        if(args.Length==1 && args[0]=="--check-package"){
            try{Package.Read(AppDomain.CurrentDomain.BaseDirectory).Command("COM12");return 0;}catch{return 2;}
        }
        Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new Installer());return 0;
    }
}
