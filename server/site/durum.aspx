<%@ Page Language="C#" CodePage="65001" %>
<%@ Import Namespace="EvseBulut" %>
<script runat="server">
CihazKaydi cihaz;
string kod = "";
int yenile;
void Page_Load() {
    Ortam.Hazirla();
    kod = (Request.QueryString["kod"] ?? "").Trim().ToUpperInvariant();
    if (Jeton.KodGecerli(kod)) cihaz = Depo.Yukle(kod);
    if (cihaz != null) yenile = 5;
}
string CanliKwh() {
    if (cihaz == null || cihaz.Acik == null) return "—";
    double k = cihaz.Acik.ESon - cihaz.Acik.EBas;
    if (k < 0) k = 0;
    return Arayuz.Sayi(k, 2) + " kWh";
}
</script>
<% Response.Write(EvseBulut.Arayuz.Ust("İstasyon durumu", yenile)); %>
<h1>İstasyon durumu</h1>
<p class="not">Bu sayfa, kartın kendi ekranının uzaktan görünüşüdür. Kartın yanındayken uygulamayı <b>http://evse.local/</b> adresinden açın. Kare kod o yerel adrese gider.</p>
<% if (!EvseBulut.Jeton.KodGecerli(kod)) { %>
<div class="kart">
  <form method="get" action="durum.aspx">
    <label for="kod">İstasyon kodu</label>
    <input id="kod" name="kod" type="text" maxlength="16" autocapitalize="characters" spellcheck="false">
    <p><button type="submit">Göster</button></p>
  </form>
</div>
<% } else if (cihaz == null) { %>
<p class="hata">Bu kodla kayıtlı istasyon yok.</p>
<% } else {
     bool canli = EvseBulut.Arayuz.Cevrimici(cihaz.SonPollUtc, DateTime.UtcNow);
     string st = cihaz.Son == null ? "" : cihaz.Son.State;
%>
<div class="kart">
  <p><span class="rozet <%= canli ? "canli" : "kopuk" %>"><%= canli ? "Çevrimiçi" : "Çevrimdışı" %></span></p>
  <p class="durum"><%= EvseBulut.Arayuz.E(EvseBulut.Arayuz.DurumAdi(st)) %></p>
  <p><%= EvseBulut.Arayuz.E(cihaz.Ad) %> · kod <%= EvseBulut.Arayuz.E(cihaz.Kod) %></p>
  <p class="not">Son yoklama: <%= EvseBulut.Arayuz.E(EvseBulut.Arayuz.TrYazi(cihaz.SonPollUtc)) %></p>
  <% if (cihaz.Son != null && cihaz.Son.Paused) { %>
  <p class="uyari">Planlı şarj bu aralıkta bekletiyor.<% if (cihaz.Son.ResumeInSec >= 0) { %> Yaklaşık <%= EvseBulut.Arayuz.E(EvseBulut.Arayuz.Sure(cihaz.Son.ResumeInSec)) %> sonra.<% } %></p>
  <% } %>
  <% if (cihaz.Son != null && (st == "C" || st == "D")) { %>
  <p>Güç <b><%= EvseBulut.Arayuz.Sayi(cihaz.Son.PW / 1000.0, 1) %> kW</b></p>
  <p>Akım <%= EvseBulut.Arayuz.Sayi(cihaz.Son.Ia, 1) %> / <%= EvseBulut.Arayuz.Sayi(cihaz.Son.Ib, 1) %> / <%= EvseBulut.Arayuz.Sayi(cihaz.Son.Ic, 1) %> A</p>
  <% } %>
  <p>Bu seans: <%= CanliKwh() %></p>
</div>
<div class="kart">
  <h2>Son seanslar</h2>
  <% if (cihaz.Seanslar == null || cihaz.Seanslar.Count == 0) { %>
  <p class="not">Henüz kayıtlı seans yok. Seanslar bu sunucuda durur; kartın kendi belleğindeki seanslardan ayrıdır.</p>
  <% } else { %>
  <table>
    <tr><th>Başlangıç</th><th>Süre</th><th>Enerji</th></tr>
    <% int n = Math.Min(5, cihaz.Seanslar.Count); for (int i = 0; i < n; i++) { var s = cihaz.Seanslar[i]; %>
    <tr>
      <td><%= EvseBulut.Arayuz.E(EvseBulut.Arayuz.TrYazi(s.BasUtc)) %></td>
      <td><%= EvseBulut.Arayuz.E(EvseBulut.Arayuz.Sure(s.Sn)) %></td>
      <td><%= EvseBulut.Arayuz.Sayi(s.KWh, 2) %> kWh</td>
    </tr>
    <% } %>
  </table>
  <% } %>
</div>
<% } %>
<p><a href="default.aspx">Başa dön</a></p>
<% Response.Write(EvseBulut.Arayuz.Alt()); %>
