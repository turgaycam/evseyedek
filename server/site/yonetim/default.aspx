<%@ Page Language="C#" CodePage="65001" %>
<%@ Import Namespace="System.Collections.Generic" %>
<%@ Import Namespace="EvseBulut" %>
<script runat="server">
string mesaj = "";
string hata = "";
List<CihazKaydi> liste;
void Page_Load() {
    Ortam.Hazirla();
    if (!Arayuz.Yonetici(Session)) { Response.Redirect("giris.aspx"); return; }
    string csrf = Arayuz.CsrfAl(Session);
    if (string.Equals(Request.HttpMethod, "POST", StringComparison.OrdinalIgnoreCase)) {
        if (!Arayuz.CsrfTamam(Request, Session)) { hata = "Oturum doğrulanamadı. Sayfayı yenileyin."; }
        else {
            try {
                string ad = Request.Form["ad"];
                string kod = Request.Form["kod"];
                string secret;
                string olusan = Depo.CihazOlustur(ad, kod, out secret);
                Session["yeniSecret"] = secret;
                Session["yeniKod"] = olusan;
                Response.Redirect("cihaz.aspx?kod=" + HttpUtility.UrlEncode(olusan) + "&yeni=1", false);
                Context.ApplicationInstance.CompleteRequest();
                return;
            } catch (ArgumentException) { hata = "Kod 3–16 karakter olmalı: büyük harf, rakam, _ veya -."; }
            catch (InvalidOperationException) { hata = "Bu kod zaten kayıtlı."; }
            catch (Exception) { hata = "Kayıt yazılamadı. App_Data yazma iznini kontrol edin."; }
        }
    }
    liste = Depo.Liste();
}
</script>
<% Response.Write(EvseBulut.Arayuz.Ust("İstasyonlar")); %>
<h1>İstasyonlar</h1>
<p class="not">Kartın yönetim panelinde bulut adresi <b>https://turgay.rotosis.com</b> olur. Sondaki bölü işareti kart tarafından atılır. Kart, sunucu kapalıyken de şarj eder.</p>
<p class="satir"><a class="dugme ik" href="cikis.aspx">Çıkış</a></p>
<% if (hata.Length > 0) { %><p class="hata"><%= EvseBulut.Arayuz.E(hata) %></p><% } %>
<% if (mesaj.Length > 0) { %><p class="iyi"><%= EvseBulut.Arayuz.E(mesaj) %></p><% } %>
<div class="kart">
  <table>
    <tr><th>Ad</th><th>Kod</th><th>Durum</th><th>Son yoklama</th></tr>
    <% if (liste == null || liste.Count == 0) { %>
    <tr><td colspan="4">Henüz istasyon yok.</td></tr>
    <% } else { foreach (CihazKaydi c in liste) {
         bool canli = Arayuz.Cevrimici(c.SonPollUtc, DateTime.UtcNow);
         string st = c.Son == null ? "—" : Arayuz.DurumAdi(c.Son.State);
    %>
    <tr>
      <td><a href="cihaz.aspx?kod=<%= EvseBulut.Arayuz.E(c.Kod) %>"><%= EvseBulut.Arayuz.E(c.Ad) %></a></td>
      <td class="gizli"><%= EvseBulut.Arayuz.E(c.Kod) %></td>
      <td><span class="rozet <%= canli ? "canli" : "kopuk" %>"><%= canli ? "çevrimiçi" : "çevrimdışı" %></span> <%= EvseBulut.Arayuz.E(st) %></td>
      <td><%= EvseBulut.Arayuz.E(Arayuz.TrYazi(c.SonPollUtc)) %></td>
    </tr>
    <% } } %>
  </table>
</div>
<div class="kart">
  <h2>Yeni istasyon</h2>
  <form method="post" action="default.aspx">
    <input type="hidden" name="csrf" value="<%= EvseBulut.Arayuz.E(EvseBulut.Arayuz.CsrfAl(Session)) %>">
    <label for="ad">Görünen ad</label>
    <input id="ad" name="ad" type="text" maxlength="40" placeholder="Ev">
    <label for="kod">İstasyon kodu (boşsa otomatik)</label>
    <input id="kod" name="kod" type="text" maxlength="16" autocapitalize="characters" spellcheck="false" placeholder="AB12CD">
    <p><button type="submit">Oluştur</button></p>
  </form>
  <p class="not">Gizli anahtar yalnızca oluşturma anında bir kez gösterilir. Karttaki anahtar alanına yapıştırın. 43 karakterdir.</p>
</div>
<% Response.Write(EvseBulut.Arayuz.Alt()); %>
