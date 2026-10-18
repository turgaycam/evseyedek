using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Web.Script.Serialization;

namespace EvseBulut
{
    // Kart firmware'i (src/net/cloud_core.cpp) ile ayni bicim.
    // Istek: POST {adres}/api/device/poll
    // Yanit en fazla 1024 bayt: {"ok":true,"t":<unix>,"cmd":null}
    // veya {"ok":true,"t":<unix>,"cmd":{"id":<pozitif>,"type":"start|stop|set_limit"}}

    public static class Parola
    {
        public const int Tur = 20000;

        public static string Ozetle(string duz)
        {
            byte[] tuz = new byte[16];
            using (var rng = RandomNumberGenerator.Create()) rng.GetBytes(tuz);
            byte[] ozet = Ture(duz, tuz, Tur);
            return "pbkdf2-sha256$" + Tur.ToString(CultureInfo.InvariantCulture) + "$" +
                   Convert.ToBase64String(tuz) + "$" + Convert.ToBase64String(ozet);
        }

        public static bool Dogrula(string duz, string kayitli)
        {
            if (string.IsNullOrEmpty(duz) || string.IsNullOrEmpty(kayitli)) return false;
            string[] p = kayitli.Split('$');
            if (p.Length != 4 || p[0] != "pbkdf2-sha256") return false;
            int tur;
            if (!int.TryParse(p[1], NumberStyles.None, CultureInfo.InvariantCulture, out tur) || tur < 1000 || tur > 200000) return false;
            byte[] tuz, beklenen;
            try
            {
                tuz = Convert.FromBase64String(p[2]);
                beklenen = Convert.FromBase64String(p[3]);
            }
            catch (FormatException) { return false; }
            byte[] gelen = Ture(duz, tuz, tur);
            return SabitEsit(gelen, beklenen);
        }

        static byte[] Ture(string duz, byte[] tuz, int tur)
        {
            using (var pbk = new Rfc2898DeriveBytes(duz, tuz, tur, HashAlgorithmName.SHA256))
                return pbk.GetBytes(32);
        }

        static bool SabitEsit(byte[] a, byte[] b)
        {
            if (a == null || b == null || a.Length != b.Length) return false;
            int f = 0;
            for (int i = 0; i < a.Length; i++) f |= a[i] ^ b[i];
            return f == 0;
        }
    }

    public static class Jeton
    {
        public static bool KodGecerli(string s)
        {
            if (string.IsNullOrEmpty(s) || s.Length < 3 || s.Length > 16) return false;
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                bool ok = (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
                if (!ok) return false;
            }
            return true;
        }

        public static bool SecretGecerli(string s)
        {
            if (s == null || s.Length != 43) return false;
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
                if (!ok) return false;
            }
            return true;
        }

        public static string SecretUret()
        {
            byte[] ham = new byte[32];
            using (var rng = RandomNumberGenerator.Create()) rng.GetBytes(ham);
            string b64 = Convert.ToBase64String(ham).TrimEnd('=').Replace('+', '-').Replace('/', '_');
            if (!SecretGecerli(b64)) throw new InvalidOperationException("secret uzunlugu");
            return b64;
        }

        public static string KodUret()
        {
            const string alfabe = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
            byte[] ham = new byte[6];
            using (var rng = RandomNumberGenerator.Create()) rng.GetBytes(ham);
            var sb = new StringBuilder(6);
            for (int i = 0; i < ham.Length; i++) sb.Append(alfabe[ham[i] % alfabe.Length]);
            return sb.ToString();
        }
    }

    public sealed class YoklamaIstegi
    {
        public string Code;
        public string Secret;
        public char State;
        public double Ia, Ib, Ic, PW, EKWh;
        public long TSec;
        public string Fw;
        public bool Paused;
        public string PauseReason;
        public long ResumeInSec = -1;
        public bool SchedOn, SchedBlocked, SchedTimeOk;
        public long SchedNext = -1;
        public bool HasAck;
        public long AckId;
        public bool AckOk;
    }

    public static class Protokol
    {
        public static bool IstekOku(string json, out YoklamaIstegi istek, out string hata)
        {
            istek = null;
            hata = null;
            if (string.IsNullOrEmpty(json) || json.Length > 8192) { hata = "boyut"; return false; }
            object raw;
            try { raw = new JavaScriptSerializer().DeserializeObject(json); }
            catch (Exception) { hata = "json"; return false; }
            var d = raw as Dictionary<string, object>;
            if (d == null) { hata = "json"; return false; }

            var y = new YoklamaIstegi();
            y.Code = Metin(d, "code");
            y.Secret = Metin(d, "secret");
            string st = Metin(d, "state");
            if (!Jeton.KodGecerli(y.Code) || !Jeton.SecretGecerli(y.Secret) || st == null || st.Length != 1) { hata = "kimlik"; return false; }
            char ch = st[0];
            y.State = (ch >= 'A' && ch <= 'F') ? ch : 'F';
            y.Ia = Sayi(d, "ia");
            y.Ib = Sayi(d, "ib");
            y.Ic = Sayi(d, "ic");
            y.PW = Sayi(d, "pW");
            y.EKWh = Sayi(d, "eKWh");
            y.TSec = Tam(d, "tSec");
            y.Fw = Metin(d, "fw") ?? "";
            if (y.Fw.Length > 24) y.Fw = y.Fw.Substring(0, 24);
            y.Paused = Bayrak(d, "paused");
            y.PauseReason = Metin(d, "pauseReason");
            y.ResumeInSec = BosVeyaTam(d, "resumeInSec");
            var sched = d.ContainsKey("sched") ? d["sched"] as Dictionary<string, object> : null;
            if (sched != null)
            {
                y.SchedOn = Bayrak(sched, "on");
                y.SchedBlocked = Bayrak(sched, "blocked");
                y.SchedTimeOk = Bayrak(sched, "timeOk");
                y.SchedNext = BosVeyaTam(sched, "nextChangeInSec");
            }
            if (d.ContainsKey("ackId") && d["ackId"] != null)
            {
                long id;
                if (!TryTam(d["ackId"], out id) || id <= 0) { hata = "ack"; return false; }
                y.HasAck = true;
                y.AckId = id;
                y.AckOk = Bayrak(d, "ackOk");
            }
            istek = y;
            return true;
        }

        public static string YanitYaz(long unix, long cmdId, string cmdType)
        {
            if (unix < 0) unix = 0;
            string govde;
            if (cmdId <= 0 || string.IsNullOrEmpty(cmdType))
                govde = "{\"ok\":true,\"t\":" + unix.ToString(CultureInfo.InvariantCulture) + ",\"cmd\":null}";
            else
            {
                if (cmdType != "start" && cmdType != "stop" && cmdType != "set_limit")
                    throw new ArgumentException("komut");
                govde = "{\"ok\":true,\"t\":" + unix.ToString(CultureInfo.InvariantCulture) +
                        ",\"cmd\":{\"id\":" + cmdId.ToString(CultureInfo.InvariantCulture) +
                        ",\"type\":\"" + cmdType + "\"}}";
            }
            if (Encoding.UTF8.GetByteCount(govde) > 1024) throw new InvalidOperationException("yanit 1024 bayti asiyor");
            return govde;
        }

        static string Metin(Dictionary<string, object> d, string anahtar)
        {
            if (!d.ContainsKey(anahtar) || d[anahtar] == null) return null;
            return Convert.ToString(d[anahtar], CultureInfo.InvariantCulture);
        }

        static bool Bayrak(Dictionary<string, object> d, string anahtar)
        {
            if (!d.ContainsKey(anahtar) || d[anahtar] == null) return false;
            object o = d[anahtar];
            if (o is bool) return (bool)o;
            return false;
        }

        static double Sayi(Dictionary<string, object> d, string anahtar)
        {
            if (!d.ContainsKey(anahtar) || d[anahtar] == null) return 0;
            try { return Convert.ToDouble(d[anahtar], CultureInfo.InvariantCulture); }
            catch (Exception) { return 0; }
        }

        static long Tam(Dictionary<string, object> d, string anahtar)
        {
            if (!d.ContainsKey(anahtar) || d[anahtar] == null) return 0;
            long v;
            return TryTam(d[anahtar], out v) ? v : 0;
        }

        static long BosVeyaTam(Dictionary<string, object> d, string anahtar)
        {
            if (!d.ContainsKey(anahtar) || d[anahtar] == null) return -1;
            long v;
            return TryTam(d[anahtar], out v) ? v : -1;
        }

        static bool TryTam(object o, out long v)
        {
            v = 0;
            if (o == null) return false;
            if (o is int) { v = (int)o; return true; }
            if (o is long) { v = (long)o; return true; }
            if (o is decimal) { v = (long)(decimal)o; return true; }
            if (o is double) { v = (long)(double)o; return true; }
            return long.TryParse(Convert.ToString(o, CultureInfo.InvariantCulture), NumberStyles.Integer, CultureInfo.InvariantCulture, out v);
        }
    }

    public sealed class Telemetri
    {
        public string State { get; set; }
        public double Ia { get; set; }
        public double Ib { get; set; }
        public double Ic { get; set; }
        public double PW { get; set; }
        public double EKWh { get; set; }
        public long TSec { get; set; }
        public string Fw { get; set; }
        public bool Paused { get; set; }
        public string PauseReason { get; set; }
        public long ResumeInSec { get; set; }
        public bool SchedOn { get; set; }
        public bool SchedBlocked { get; set; }
        public bool SchedTimeOk { get; set; }
        public long SchedNext { get; set; }
    }

    public sealed class KomutKaydi
    {
        public long Id { get; set; }
        public string Type { get; set; }
    }

    public sealed class AckKaydi
    {
        public long Id { get; set; }
        public bool Ok { get; set; }
        public string ZamanUtc { get; set; }
    }

    public sealed class AcikSeans
    {
        public string BasUtc { get; set; }
        public double EBas { get; set; }
        public double ESon { get; set; }
        public long TBas { get; set; }
        public long TSon { get; set; }
        public bool Duraklatildi { get; set; }
        public string SonDurum { get; set; }
    }

    public sealed class SeansKaydi
    {
        public string BasUtc { get; set; }
        public string BitUtc { get; set; }
        public double KWh { get; set; }
        public long Sn { get; set; }
        public string SonDurum { get; set; }
    }

    public sealed class CihazKaydi
    {
        public string Kod { get; set; }
        public string Ad { get; set; }
        public string SecretOzet { get; set; }
        public string OlusturmaUtc { get; set; }
        public string SonPollUtc { get; set; }
        public Telemetri Son { get; set; }
        public KomutKaydi Bekleyen { get; set; }
        public string Siradaki { get; set; }
        public long SonId { get; set; }
        public AckKaydi SonAck { get; set; }
        public string SonNot { get; set; }
        public AcikSeans Acik { get; set; }
        public List<SeansKaydi> Seanslar { get; set; }

        public CihazKaydi()
        {
            Seanslar = new List<SeansKaydi>();
        }
    }

    public static class Seans
    {
        public static void Guncelle(CihazKaydi c, YoklamaIstegi y, DateTime utc)
        {
            if (c.Seanslar == null) c.Seanslar = new List<SeansKaydi>();
            if (c.Acik != null && y.EKWh + 0.0001 < c.Acik.ESon) Kapat(c, utc);
            if (y.State == 'C' || y.State == 'D')
            {
                if (c.Acik == null)
                {
                    c.Acik = new AcikSeans
                    {
                        BasUtc = Iso(utc),
                        EBas = y.EKWh,
                        ESon = y.EKWh,
                        TBas = y.TSec,
                        TSon = y.TSec,
                        Duraklatildi = false,
                        SonDurum = y.State.ToString()
                    };
                }
                else
                {
                    c.Acik.ESon = y.EKWh;
                    c.Acik.TSon = y.TSec;
                    c.Acik.Duraklatildi = false;
                    c.Acik.SonDurum = y.State.ToString();
                }
            }
            else if (c.Acik != null && y.State == 'B')
            {
                c.Acik.Duraklatildi = true;
                c.Acik.SonDurum = "B";
            }
            else if (c.Acik != null && (y.State == 'A' || y.State == 'E' || y.State == 'F'))
            {
                Kapat(c, utc);
            }
        }

        static void Kapat(CihazKaydi c, DateTime utc)
        {
            AcikSeans a = c.Acik;
            c.Acik = null;
            if (a == null) return;
            double kwh = a.ESon - a.EBas;
            if (kwh < 0) kwh = 0;
            long sn = a.TSon - a.TBas;
            if (sn < 0) sn = 0;
            DateTime bas;
            double duvar = 0;
            if (DateTime.TryParse(a.BasUtc, CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind, out bas))
                duvar = (utc - bas.ToUniversalTime()).TotalSeconds;
            if (kwh < 0.001 && sn < 5 && duvar < 20) return;
            c.Seanslar.Insert(0, new SeansKaydi
            {
                BasUtc = a.BasUtc,
                BitUtc = Iso(utc),
                KWh = Math.Round(kwh, 3),
                Sn = sn > 0 ? sn : (long)Math.Max(0, duvar),
                SonDurum = a.SonDurum
            });
            if (c.Seanslar.Count > 100) c.Seanslar.RemoveRange(100, c.Seanslar.Count - 100);
        }

        public static string Iso(DateTime utc)
        {
            return utc.ToUniversalTime().ToString("o", CultureInfo.InvariantCulture);
        }
    }

    public static class Kuyruk
    {
        public static void Iste(CihazKaydi c, string tip)
        {
            if (tip != "start" && tip != "stop") throw new ArgumentException("komut");
            if (c.Bekleyen == null)
            {
                c.SonId++;
                if (c.SonId <= 0) c.SonId = 1;
                c.Bekleyen = new KomutKaydi { Id = c.SonId, Type = tip };
                c.Siradaki = null;
                c.SonNot = "Komut yolda: " + tip;
                return;
            }
            if (c.Bekleyen.Type == tip) { c.Siradaki = null; return; }
            c.Siradaki = tip;
            c.SonNot = "Onceki komut onaylaninca " + tip + " gidecek";
        }

        public static string Isle(CihazKaydi c, YoklamaIstegi y, DateTime utc)
        {
            if (y.HasAck && c.Bekleyen != null && y.AckId == c.Bekleyen.Id)
            {
                c.SonAck = new AckKaydi { Id = y.AckId, Ok = y.AckOk, ZamanUtc = Seans.Iso(utc) };
                string tip = c.Bekleyen.Type;
                c.Bekleyen = null;
                c.SonNot = y.AckOk ? ("Kart uyguladi: " + tip) : ("Kart reddetti: " + tip);
                if (c.Siradaki == "start" || c.Siradaki == "stop")
                {
                    string sonraki = c.Siradaki;
                    c.Siradaki = null;
                    Iste(c, sonraki);
                }
            }
            Seans.Guncelle(c, y, utc);
            c.SonPollUtc = Seans.Iso(utc);
            c.Son = new Telemetri
            {
                State = y.State.ToString(),
                Ia = y.Ia,
                Ib = y.Ib,
                Ic = y.Ic,
                PW = y.PW,
                EKWh = y.EKWh,
                TSec = y.TSec,
                Fw = y.Fw,
                Paused = y.Paused,
                PauseReason = y.PauseReason,
                ResumeInSec = y.ResumeInSec,
                SchedOn = y.SchedOn,
                SchedBlocked = y.SchedBlocked,
                SchedTimeOk = y.SchedTimeOk,
                SchedNext = y.SchedNext
            };
            long id = 0;
            string type = null;
            if (c.Bekleyen != null && c.Bekleyen.Id > 0)
            {
                id = c.Bekleyen.Id;
                type = c.Bekleyen.Type;
            }
            long unix = (long)(utc.ToUniversalTime() - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
            return Protokol.YanitYaz(unix, id, type);
        }
    }

    public static class Depo
    {
        static readonly object Kilit = new object();
        static readonly string SahteOzet = Parola.Ozetle("karsilastirma-icin-sabit-degil");
        static readonly JavaScriptSerializer Ser = new JavaScriptSerializer { MaxJsonLength = 1024 * 1024 };
        static readonly UTF8Encoding Utf8 = new UTF8Encoding(false);

        public static string Kok { get; set; }

        public static void Hazirla(string kok)
        {
            if (string.IsNullOrEmpty(kok)) throw new InvalidOperationException("veri klasoru yok");
            Directory.CreateDirectory(kok);
            Directory.CreateDirectory(Path.Combine(kok, "cihazlar"));
            Kok = kok;
        }

        public static bool Kuruldu()
        {
            return YoneticiOku() != null;
        }

        public static void YoneticiYaz(string kullanici, string parola)
        {
            if (string.IsNullOrEmpty(kullanici) || kullanici.Length > 32) throw new ArgumentException("kullanici");
            if (parola == null || parola.Length < 10 || parola.Length > 64) throw new ArgumentException("parola");
            var kayit = new Dictionary<string, string>
            {
                { "kullanici", kullanici },
                { "ozet", Parola.Ozetle(parola) },
                { "olusturma", Seans.Iso(DateTime.UtcNow) }
            };
            lock (Kilit) File.WriteAllText(YoneticiYol(), Ser.Serialize(kayit), Utf8);
        }

        public static bool YoneticiGiris(string kullanici, string parola)
        {
            var k = YoneticiOku();
            if (k == null) return false;
            string ad, ozet;
            if (!k.TryGetValue("kullanici", out ad) || !k.TryGetValue("ozet", out ozet)) return false;
            bool adDogru = string.Equals(ad, kullanici ?? "", StringComparison.Ordinal);
            bool parolaDogru = Parola.Dogrula(parola ?? "", ozet);
            return adDogru && parolaDogru;
        }

        public static string CihazOlustur(string ad, string kod, out string secretDuz)
        {
            ad = (ad ?? "").Trim();
            if (ad.Length == 0) ad = "Sarj istasyonu";
            if (ad.Length > 40) ad = ad.Substring(0, 40);
            kod = string.IsNullOrEmpty(kod) ? "" : kod.Trim().ToUpperInvariant();
            lock (Kilit)
            {
                Klasor();
                if (kod.Length == 0)
                {
                    for (int i = 0; i < 8 && (kod.Length == 0 || File.Exists(CihazYol(kod))); i++) kod = Jeton.KodUret();
                }
                if (!Jeton.KodGecerli(kod)) throw new ArgumentException("kod");
                if (File.Exists(CihazYol(kod))) throw new InvalidOperationException("kod var");
                secretDuz = Jeton.SecretUret();
                var c = new CihazKaydi
                {
                    Kod = kod,
                    Ad = ad,
                    SecretOzet = Parola.Ozetle(secretDuz),
                    OlusturmaUtc = Seans.Iso(DateTime.UtcNow),
                    Seanslar = new List<SeansKaydi>()
                };
                Yaz(c);
                return kod;
            }
        }

        public static List<CihazKaydi> Liste()
        {
            lock (Kilit)
            {
                var liste = new List<CihazKaydi>();
                string dir = Path.Combine(KokGerekli(), "cihazlar");
                if (!Directory.Exists(dir)) return liste;
                foreach (string f in Directory.GetFiles(dir, "*.json"))
                {
                    CihazKaydi c = OkuDosya(f);
                    if (c != null) liste.Add(c);
                }
                liste.Sort(delegate(CihazKaydi a, CihazKaydi b) { return string.Compare(a.Ad, b.Ad, StringComparison.CurrentCulture); });
                return liste;
            }
        }

        public static CihazKaydi Yukle(string kod)
        {
            if (!Jeton.KodGecerli(kod)) return null;
            lock (Kilit) return OkuDosya(CihazYol(kod));
        }

        public static void Sil(string kod)
        {
            if (!Jeton.KodGecerli(kod)) return;
            lock (Kilit)
            {
                string p = CihazYol(kod);
                if (File.Exists(p)) File.Delete(p);
            }
        }

        public static void KomutEkle(string kod, string tip)
        {
            lock (Kilit)
            {
                CihazKaydi c = OkuDosya(CihazYol(kod));
                if (c == null) throw new InvalidOperationException("cihaz yok");
                Kuyruk.Iste(c, tip);
                Yaz(c);
            }
        }

        public static string Yokla(YoklamaIstegi y, DateTime utc, out int http)
        {
            http = 401;
            lock (Kilit)
            {
                CihazKaydi c = Jeton.KodGecerli(y.Code) ? OkuDosya(CihazYol(y.Code)) : null;
                string ozet = c == null ? SahteOzet : c.SecretOzet;
                bool gizli = Parola.Dogrula(y.Secret, ozet);
                if (c == null || !gizli) return "{\"ok\":false}";
                string yanit = Kuyruk.Isle(c, y, utc);
                Yaz(c);
                http = 200;
                return yanit;
            }
        }

        static Dictionary<string, string> YoneticiOku()
        {
            string p = YoneticiYol();
            if (!File.Exists(p)) return null;
            object raw = Ser.DeserializeObject(File.ReadAllText(p, Utf8));
            var d = raw as Dictionary<string, object>;
            if (d == null) return null;
            var s = new Dictionary<string, string>();
            foreach (var kv in d) s[kv.Key] = kv.Value == null ? "" : Convert.ToString(kv.Value, CultureInfo.InvariantCulture);
            return s;
        }

        static void Yaz(CihazKaydi c)
        {
            if (c.Seanslar == null) c.Seanslar = new List<SeansKaydi>();
            string json = Ser.Serialize(c);
            string yol = CihazYol(c.Kod);
            string tmp = yol + ".tmp";
            File.WriteAllText(tmp, json, Utf8);
            if (File.Exists(yol)) File.Delete(yol);
            File.Move(tmp, yol);
        }

        static CihazKaydi OkuDosya(string yol)
        {
            if (!File.Exists(yol)) return null;
            CihazKaydi c = Ser.Deserialize<CihazKaydi>(File.ReadAllText(yol, Utf8));
            if (c == null) return null;
            if (c.Seanslar == null) c.Seanslar = new List<SeansKaydi>();
            return c;
        }

        static void Klasor()
        {
            Directory.CreateDirectory(Path.Combine(KokGerekli(), "cihazlar"));
        }

        static string KokGerekli()
        {
            if (string.IsNullOrEmpty(Kok)) throw new InvalidOperationException("App_Data hazir degil");
            return Kok;
        }

        static string YoneticiYol()
        {
            return Path.Combine(KokGerekli(), "yonetici.json");
        }

        static string CihazYol(string kod)
        {
            if (!Jeton.KodGecerli(kod)) throw new ArgumentException("kod");
            return Path.Combine(KokGerekli(), "cihazlar", kod + ".json");
        }
    }
}
