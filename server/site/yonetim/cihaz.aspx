<%@ Page Language="C#" CodePage="65001" %>
<%@ Import Namespace="EvseBulut" %>
<script runat="server">
CihazKaydi cihaz;
string kod = "";
string hata = "";
string yeniSecret = "";
void Page_Load() {
    Ortam.Hazirla();
    if (!Arayuz.Yonetici(Session)) { Response.Redirect("giris.aspx"); return; }
    Arayuz.CsrfAl(Session);
    kod = (Request.QueryString["kod"] ?? "").Trim().ToUpperInvariant();
    if (Request.QueryString["yeni"] == "1") {
        string sk = Session["yeniKod"] as string;
        if (sk == kod) {
            yeniSecret = Session["yeniSecret"] as string ?? "";
            Session.Remove("yeniSecret");
            Session.Remove("yeniKod");
        }
    }
    if (!Jeton.KodGecerli(kod)) { hata = "Kod geçersiz."; return; }
    if (string.Equals(Request.HttpMethod, "POST", StringComparison.OrdinalIgnoreCase)) {
        if (!Arayuz.CsrfTamam(Request, Session)) hata = "Oturum doğrulanamadı. Sayfayı yenileyin.";
        else {
            string islem = Request.Form["islem"] ?? "";
            try {
                if (islem == "start" || islem == "stop") Depo.KomutEkle(kod, islem);
                else if (islem == "sil") {
                    string onay = (Request.Form["onay"] ?? "").Trim().ToUpperInvariant();
                    if (onay != kod) hata = "Silmek için istasyon kodunu aynen yazın.";
                    else {
                        Depo.Sil(kod);
                        Response.Redirect("default.aspx", false);
                        Context.ApplicationInstance.CompleteRequest();
                        return;
                    }
                }
            } catch (Exception) { hata = "İşlem yazılamadı."; }
        }
    }
    cihaz = Depo.Yukle(kod);
    if (cihaz == null && hata.Length == 0) hata = "İstasyon yok.";
}
</script>
<% Response.Write(EvseBulut.Arayuz.Ust("İstasyon")); %>
<p><a href="default.aspx">İstasyonlar</a></p>
<% if (hata.Length > 0) { %><p class="hata"><%= EvseBulut.Arayuz.E(hata) %></p><% } %>
<% if (yeniSecret.Length > 0) { %>
<div class="uyari">
  <p><b>Gizli anahtarı şimdi kopyalayın.</b> Bu sayfadan çıkınca bir daha gösterilmez.</p>
  <p class="gizli"><%= EvseBulut.Arayuz.E(yeniSecret) %></p>
</div>
<% } %>
<% if (cihaz != null) {
     bool canli = Arayuz.Cevrimici(cihaz.SonPollUtc, DateTime.UtcNow);
     string st = cihaz.Son == null ? "" : cihaz.Son.State;
     string durumUrl = ResolveUrl("~/durum.aspx") + "?kod=" + HttpUtility.UrlEncode(cihaz.Kod);
%>
<h1><%= EvseBulut.Arayuz.E(cihaz.Ad) %></h1>
<p><span class="rozet <%= canli ? "canli" : "kopuk" %>"><%= canli ? "Çevrimiçi" : "Çevrimdışı" %></span>
 kod <span class="gizli"><%= EvseBulut.Arayuz.E(cihaz.Kod) %></span></p>
<div class="kart">
  <p class="durum"><%= EvseBulut.Arayuz.E(Arayuz.DurumAdi(st)) %></p>
  <p class="not">Son yoklama: <%= EvseBulut.Arayuz.E(Arayuz.TrYazi(cihaz.SonPollUtc)) %><% if (cihaz.Son != null && !string.IsNullOrEmpty(cihaz.Son.Fw)) { %> · yazılım <%= EvseBulut.Arayuz.E(cihaz.Son.Fw) %><% } %></p>
  <% if (!string.IsNullOrEmpty(cihaz.SonNot)) { %><p><%= EvseBulut.Arayuz.E(cihaz.SonNot) %></p><% } %>
  <% if (cihaz.Bekleyen != null) { %><p class="uyari">Bekleyen komut #<%= cihaz.Bekleyen.Id %> <%= EvseBulut.Arayuz.E(cihaz.Bekleyen.Type) %>. Kart onaylayana kadar aynı numara yinelenir.</p><% } %>
  <% if (cihaz.Son != null) { %>
  <p>Güç <%= Arayuz.Sayi(cihaz.Son.PW / 1000.0, 1) %> kW · akım <%= Arayuz.Sayi(cihaz.Son.Ia, 1) %> / <%= Arayuz.Sayi(cihaz.Son.Ib, 1) %> / <%= Arayuz.Sayi(cihaz.Son.Ic, 1) %> A</p>
  <p>Kartın açılıştan beri saydığı enerji: <%= Arayuz.Sayi(cihaz.Son.EKWh, 3) %> kWh</p>
  <% if (cihaz.Son.SchedOn) { %><p class="not">Planlı şarj açık<%= cihaz.Son.SchedBlocked ? ", şu an yasaklı aralıkta" : "" %><%= cihaz.Son.SchedTimeOk ? "" : ", kart saati henüz yok" %>.</p><% } %>
  <% } %>
  <form method="post" action="cihaz.aspx?kod=<%= EvseBulut.Arayuz.E(cihaz.Kod) %>" class="satir">
    <input type="hidden" name="csrf" value="<%= EvseBulut.Arayuz.E(Arayuz.CsrfAl(Session)) %>">
    <button type="submit" name="islem" value="start">Başlat</button>
    <button type="submit" name="islem" value="stop" class="tehlike">Durdur</button>
  </form>
  <p class="not">Başlat ve Durdur, kart bir sonraki yoklamada (yaklaşık 2,5 sn) uygulanır. Kart <b>set_limit</b> komutunu alır ama uygulamaz; bu panel onu göndermez. “Şarjı yalnızca uygulamadan başlat” kapalıysa kart, komut gelmeden de araç takılınca şarj eder. Açıksa şarjın başlaması için Başlat gerekir; başladıktan sonra bu sunucu kesilse de şarj sürmeye devam eder.</p>
</div>
<div class="kart">
  <h2>Herkese açık sayfa</h2>
  <p class="gizli"><%= EvseBulut.Arayuz.E(new Uri(Request.Url, durumUrl).AbsoluteUri) %></p>
  <p class="not">Kartın kare kodu bu adrese gitmez. Kare kod ev ağındaki http://evse.local/ adresidir.</p>
</div>
<div class="kart">
  <h2>Geçmiş seanslar</h2>
  <% if (cihaz.Seanslar == null || cihaz.Seanslar.Count == 0) { %>
  <p class="not">Kayıt yok.</p>
  <% } else { %>
  <table>
    <tr><th>Başlangıç</th><th>Bitiş</th><th>Süre</th><th>Enerji</th></tr>
    <% foreach (SeansKaydi s in cihaz.Seanslar) { %>
    <tr>
      <td><%= EvseBulut.Arayuz.E(Arayuz.TrYazi(s.BasUtc)) %></td>
      <td><%= EvseBulut.Arayuz.E(Arayuz.TrYazi(s.BitUtc)) %></td>
      <td><%= EvseBulut.Arayuz.E(Arayuz.Sure(s.Sn)) %></td>
      <td><%= Arayuz.Sayi(s.KWh, 2) %> kWh</td>
    </tr>
    <% } %>
  </table>
  <% } %>
</div>
<div class="kart">
  <h2>Kart paneline yazılacaklar</h2>
  <ul>
    <li>Bulut bağlantısı: açık</li>
    <li>Sunucu adresi: <span class="gizli">https://turgay.rotosis.com</span></li>
    <li>İstasyon kodu: <span class="gizli"><%= EvseBulut.Arayuz.E(cihaz.Kod) %></span></li>
    <li>Gizli anahtar: oluştururken kopyalanan 43 karakter</li>
  </ul>
</div>
<div class="kart">
  <h2>İstasyonu sil</h2>
  <form method="post" action="cihaz.aspx?kod=<%= EvseBulut.Arayuz.E(cihaz.Kod) %>">
    <input type="hidden" name="csrf" value="<%= EvseBulut.Arayuz.E(Arayuz.CsrfAl(Session)) %>">
    <label for="onay">Silmek için kodu yazın</label>
    <input id="onay" name="onay" type="text" maxlength="16" autocomplete="off">
    <p><button class="tehlike" type="submit" name="islem" value="sil">Sil</button></p>
  </form>
</div>
<% } %>
<% Response.Write(EvseBulut.Arayuz.Alt()); %>
