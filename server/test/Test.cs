using System;
using System.IO;
using System.Text;
using EvseBulut;

static class Test
{
    static int hata;

    static void Esit(string ad, string beklenen, string gelen)
    {
        if (beklenen != gelen)
        {
            Console.Error.WriteLine("OLMADI " + ad + "\n beklenen: " + beklenen + "\n gelen:    " + gelen);
            hata++;
        }
    }

    static void Dogru(string ad, bool kosul)
    {
        if (!kosul)
        {
            Console.Error.WriteLine("OLMADI " + ad);
            hata++;
        }
    }

    static YoklamaIstegi Ornek(char state, double kwh, bool ack, long ackId, bool ackOk)
    {
        string secret = new string('A', 43);
        var sb = new StringBuilder();
        sb.Append("{\"code\":\"4571A4\",\"secret\":\"").Append(secret).Append("\",\"state\":\"")
          .Append(state).Append("\",\"ia\":15.80,\"ib\":0.00,\"ic\":0.00,\"pW\":3630.0,\"eKWh\":")
          .Append(kwh.ToString("0.000", System.Globalization.CultureInfo.InvariantCulture))
          .Append(",\"tSec\":5321,\"fw\":\"1.87\",\"paused\":false,\"pauseReason\":null,\"resumeInSec\":null,\"sched\":{\"on\":false,\"blocked\":false,\"timeOk\":false,\"nextChangeInSec\":null}");
        if (ack) sb.Append(",\"ackId\":").Append(ackId).Append(",\"ackOk\":").Append(ackOk ? "true" : "false");
        sb.Append("}");
        YoklamaIstegi y;
        string err;
        if (!Protokol.IstekOku(sb.ToString(), out y, out err)) throw new Exception("istek " + err);
        return y;
    }

    static void Main()
    {
        Dogru("kod", Jeton.KodGecerli("4571A4") && !Jeton.KodGecerli("ab") && !Jeton.KodGecerli("ist01"));
        string secret = Jeton.SecretUret();
        Dogru("secret uret", Jeton.SecretGecerli(secret));
        Dogru("secret 43 A", Jeton.SecretGecerli(new string('A', 43)));

        Esit("bos yanit", "{\"ok\":true,\"t\":1791289200,\"cmd\":null}", Protokol.YanitYaz(1791289200, 0, null));
        Esit("start yanit", "{\"ok\":true,\"t\":1791289203,\"cmd\":{\"id\":41,\"type\":\"start\"}}", Protokol.YanitYaz(1791289203, 41, "start"));
        Esit("stop yanit", "{\"ok\":true,\"t\":1791289204,\"cmd\":{\"id\":42,\"type\":\"stop\"}}", Protokol.YanitYaz(1791289204, 42, "stop"));
        Dogru("boyut", Encoding.UTF8.GetByteCount(Protokol.YanitYaz(1791289203, 41, "start")) <= 1024);

        YoklamaIstegi y;
        string err;
        Dogru("kart json", Protokol.IstekOku(OrnekGovde(), out y, out err) && y.State == 'C' && y.Code == "4571A4" && Math.Abs(y.EKWh - 1234.567) < 0.001 && !y.HasAck);
        Dogru("kucuk kod", !Protokol.IstekOku("{\"code\":\"ab\",\"secret\":\"" + new string('A', 43) + "\",\"state\":\"A\"}", out y, out err));

        var c = new CihazKaydi();
        Kuyruk.Iste(c, "start");
        string r1 = Kuyruk.Isle(c, Ornek('C', 1.0, false, 0, false), new DateTime(2026, 10, 18, 12, 0, 0, DateTimeKind.Utc));
        Dogru("komut gitti", r1.Contains("\"type\":\"start\"") && c.Bekleyen != null);
        long id = c.Bekleyen.Id;
        string r2 = Kuyruk.Isle(c, Ornek('C', 1.2, false, 0, false), new DateTime(2026, 10, 18, 12, 0, 2, DateTimeKind.Utc));
        Dogru("ayni id", r2.Contains("\"id\":" + id.ToString()) && c.Bekleyen.Id == id);
        string r3 = Kuyruk.Isle(c, Ornek('C', 1.5, true, id, true), new DateTime(2026, 10, 18, 12, 5, 0, DateTimeKind.Utc));
        Dogru("ack temizler", r3.Contains("\"cmd\":null") && c.Bekleyen == null && c.SonAck != null && c.SonAck.Ok);

        Kuyruk.Isle(c, Ornek('A', 1.5, false, 0, false), new DateTime(2026, 10, 18, 12, 20, 0, DateTimeKind.Utc));
        Dogru("seans", c.Seanslar.Count == 1 && Math.Abs(c.Seanslar[0].KWh - 0.5) < 0.001);

        string kok = Path.Combine(Path.GetTempPath(), "evse-bulut-test-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(kok);
        Depo.Hazirla(kok);
        Dogru("kurulum bos", !Depo.Kuruldu());
        Depo.YoneticiYaz("admin", "deneme-parola");
        Dogru("giris", Depo.YoneticiGiris("admin", "deneme-parola") && !Depo.YoneticiGiris("admin", "yanlis-parola-1"));
        string duz;
        string kod = Depo.CihazOlustur("Ev", "AB12CD", out duz);
        Dogru("kod dondu", kod == "AB12CD" && Jeton.SecretGecerli(duz));
        YoklamaIstegi iyi = Ornek('A', 0, false, 0, false);
        iyi.Code = "AB12CD";
        iyi.Secret = duz;
        int http;
        string yanit = Depo.Yokla(iyi, DateTime.UtcNow, out http);
        Dogru("poll 200", http == 200 && yanit.Contains("\"cmd\":null"));
        iyi.Secret = new string('B', 43);
        Depo.Yokla(iyi, DateTime.UtcNow, out http);
        Dogru("poll 401", http == 401);
        Depo.KomutEkle("AB12CD", "stop");
        iyi.Secret = duz;
        yanit = Depo.Yokla(iyi, DateTime.UtcNow, out http);
        Dogru("stop kuyruk", http == 200 && yanit.Contains("\"type\":\"stop\""));
        string dosya = File.ReadAllText(Path.Combine(kok, "cihazlar", "AB12CD.json"));
        Dogru("dosyada duz secret yok", dosya.IndexOf(duz) < 0);

        if (hata == 0) Console.WriteLine("TAMAM");
        else Console.Error.WriteLine(hata + " hata");
        Environment.Exit(hata == 0 ? 0 : 1);
    }

    static string OrnekGovde()
    {
        return "{\"code\":\"4571A4\",\"secret\":\"" + new string('A', 43) +
               "\",\"state\":\"C\",\"ia\":15.80,\"ib\":0.00,\"ic\":0.00,\"pW\":3630.0,\"eKWh\":1234.567,\"tSec\":5321,\"fw\":\"1.87\",\"paused\":false,\"pauseReason\":null,\"resumeInSec\":null,\"sched\":{\"on\":true,\"blocked\":false,\"timeOk\":true,\"nextChangeInSec\":null}}";
    }
}
