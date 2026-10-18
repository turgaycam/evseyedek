using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Web;
using System.Web.Hosting;
using System.Web.SessionState;

namespace EvseBulut
{
    public static class Ortam
    {
        public static void Hazirla()
        {
            if (!string.IsNullOrEmpty(Depo.Kok)) return;
            string kok = HostingEnvironment.MapPath("~/App_Data");
            Depo.Hazirla(kok);
        }
    }

    public static class Arayuz
    {
        public static string E(string s)
        {
            return HttpUtility.HtmlEncode(s ?? "");
        }

        public static string Ust(string baslik)
        {
            return Ust(baslik, 0);
        }

        public static string Ust(string baslik, int yenileSaniye)
        {
            string css = VirtualPathUtility.ToAbsolute("~/site.css");
            string yenile = yenileSaniye > 0 ? "<meta http-equiv=\"refresh\" content=\"" + yenileSaniye.ToString(CultureInfo.InvariantCulture) + "\">" : "";
            return "<!DOCTYPE html><html lang=\"tr\"><head><meta charset=\"utf-8\">" +
                   "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">" + yenile +
                   "<title>" + E(baslik) + "</title><link rel=\"stylesheet\" href=\"" + E(css) + "\"></head><body><main>";
        }

        public static string Alt()
        {
            return "</main></body></html>";
        }

        public static string Sayi(double v, int basamak)
        {
            return v.ToString("F" + basamak.ToString(CultureInfo.InvariantCulture), CultureInfo.GetCultureInfo("tr-TR"));
        }

        public static string Sure(long sn)
        {
            if (sn < 0) sn = 0;
            long s = sn % 60;
            long m = (sn / 60) % 60;
            long h = sn / 3600;
            if (h > 0) return h + " sa " + m.ToString("00") + " dk";
            if (m > 0) return m + " dk " + s.ToString("00") + " sn";
            return s + " sn";
        }

        public static DateTime Tr(string iso)
        {
            DateTime utc;
            if (!DateTime.TryParse(iso, CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind, out utc))
                return DateTime.MinValue;
            if (utc.Kind == DateTimeKind.Unspecified) utc = DateTime.SpecifyKind(utc, DateTimeKind.Utc);
            utc = utc.ToUniversalTime();
            try
            {
                return TimeZoneInfo.ConvertTimeFromUtc(utc, TimeZoneInfo.FindSystemTimeZoneById("Turkey Standard Time"));
            }
            catch (TimeZoneNotFoundException) { return utc.AddHours(3); }
            catch (InvalidTimeZoneException) { return utc.AddHours(3); }
        }

        public static string TrYazi(string iso)
        {
            DateTime t = Tr(iso);
            if (t == DateTime.MinValue) return "—";
            return t.ToString("dd.MM.yyyy HH:mm", CultureInfo.GetCultureInfo("tr-TR"));
        }

        public static string DurumAdi(string state)
        {
            switch (state)
            {
                case "A": return "Araç bağlı değil";
                case "B": return "Araç bağlı, hazır";
                case "C": return "Şarj oluyor";
                case "D": return "Şarj oluyor (havalandırmalı)";
                case "E": return "Bağlantı hatası";
                case "F": return "İstasyon hatası";
                default: return "Bilinmiyor";
            }
        }

        public static bool Cevrimici(string iso, DateTime utcNow)
        {
            DateTime t;
            if (!DateTime.TryParse(iso, CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind, out t)) return false;
            if (t.Kind == DateTimeKind.Unspecified) t = DateTime.SpecifyKind(t, DateTimeKind.Utc);
            return (utcNow - t.ToUniversalTime()).TotalSeconds <= 15;
        }

        public static string CsrfAl(HttpSessionState oturum)
        {
            string t = oturum["csrf"] as string;
            if (string.IsNullOrEmpty(t))
            {
                t = Guid.NewGuid().ToString("N");
                oturum["csrf"] = t;
            }
            return t;
        }

        public static bool CsrfTamam(HttpRequest istek, HttpSessionState oturum)
        {
            string beklenen = oturum["csrf"] as string;
            string gelen = istek.Form["csrf"];
            if (string.IsNullOrEmpty(beklenen) || string.IsNullOrEmpty(gelen) || beklenen.Length != gelen.Length) return false;
            int f = 0;
            for (int i = 0; i < beklenen.Length; i++) f |= beklenen[i] ^ gelen[i];
            return f == 0;
        }

        public static bool Yonetici(HttpSessionState oturum)
        {
            return (oturum["yonetici"] as string) == "1";
        }
    }

    public static class GirisKilit
    {
        static readonly object Kilit = new object();
        static int Hata;
        static DateTime KilitBitis = DateTime.MinValue;

        public static bool Kilitli(out int saniye)
        {
            lock (Kilit)
            {
                if (DateTime.UtcNow < KilitBitis)
                {
                    saniye = (int)Math.Ceiling((KilitBitis - DateTime.UtcNow).TotalSeconds);
                    return true;
                }
                saniye = 0;
                return false;
            }
        }

        public static void Basarisiz()
        {
            lock (Kilit)
            {
                Hata++;
                if (Hata >= 5)
                {
                    KilitBitis = DateTime.UtcNow.AddMinutes(Math.Min(15, Hata - 4));
                    Hata = 0;
                }
            }
        }

        public static void Basarili()
        {
            lock (Kilit) { Hata = 0; KilitBitis = DateTime.MinValue; }
        }
    }

    public sealed class YoklamaHandler : IHttpHandler
    {
        public bool IsReusable { get { return true; } }

        public void ProcessRequest(HttpContext context)
        {
            context.Response.ContentType = "application/json; charset=utf-8";
            context.Response.ContentEncoding = Encoding.UTF8;
            context.Response.Cache.SetCacheability(HttpCacheability.NoCache);
            context.Response.Cache.SetNoStore();
            if (!string.Equals(context.Request.HttpMethod, "POST", StringComparison.OrdinalIgnoreCase))
            {
                context.Response.StatusCode = 405;
                context.Response.Write("{\"ok\":false}");
                return;
            }
            if (context.Request.ContentLength > 8192)
            {
                context.Response.StatusCode = 413;
                context.Response.Write("{\"ok\":false}");
                return;
            }
            string govde;
            using (var r = new StreamReader(context.Request.InputStream, Encoding.UTF8))
            {
                govde = r.ReadToEnd();
            }
            if (govde != null && govde.Length > 8192)
            {
                context.Response.StatusCode = 413;
                context.Response.Write("{\"ok\":false}");
                return;
            }
            YoklamaIstegi istek;
            string hata;
            if (!Protokol.IstekOku(govde, out istek, out hata))
            {
                context.Response.StatusCode = 400;
                context.Response.Write("{\"ok\":false}");
                return;
            }
            try
            {
                Ortam.Hazirla();
                int http;
                string yanit = Depo.Yokla(istek, DateTime.UtcNow, out http);
                context.Response.StatusCode = http;
                context.Response.Write(yanit);
            }
            catch (Exception)
            {
                context.Response.StatusCode = 500;
                context.Response.Write("{\"ok\":false}");
            }
        }
    }
}
