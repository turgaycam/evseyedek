<%@ Page Language="C#" CodePage="65001" %>
<script runat="server">
void Page_Load() {
    EvseBulut.Ortam.Hazirla();
}
</script>
<% Response.Write(EvseBulut.Arayuz.Ust("Evse bulut")); %>
<h1>Şarj kartının isteğe bağlı bulutu</h1>
<p class="not">Kart, bu sunucu kapalıyken de kendi Wi-Fi ağında çalışır. Telefondaki uygulama kartın adresidir: <b>http://evse.local/</b>. Bu site yalnızca siz kartın yönetim panelinde bulutu açarsanız durum tutar ve uzaktan başlat/durdur gönderir.</p>
<div class="kart">
  <h2>İstasyon durumu</h2>
  <form method="get" action="durum.aspx">
    <label for="kod">İstasyon kodu</label>
    <input id="kod" name="kod" type="text" maxlength="16" autocapitalize="characters" spellcheck="false" placeholder="Örnek: AB12CD">
    <p><button type="submit">Durumu göster</button></p>
  </form>
  <p class="not">Kod, istasyonun adıdır; gizli anahtar değildir. Şarjı buradan başlatamazsınız.</p>
</div>
<p><a href="yonetim/giris.aspx">Yönetici girişi</a></p>
<% Response.Write(EvseBulut.Arayuz.Alt()); %>
