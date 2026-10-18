<%@ Page Language="C#" CodePage="65001" ValidateRequest="true" %>
<script runat="server">
string mesaj = "";
bool bitti;
void Page_Load() {
    EvseBulut.Ortam.Hazirla();
    bitti = EvseBulut.Depo.Kuruldu();
    if (bitti || !string.Equals(Request.HttpMethod, "POST", StringComparison.OrdinalIgnoreCase)) return;
    string kullanici = (Request.Form["kullanici"] ?? "").Trim();
    string p1 = Request.Form["parola"] ?? "";
    string p2 = Request.Form["parola2"] ?? "";
    if (kullanici.Length < 3 || kullanici.Length > 32) { mesaj = "Kullanıcı adı 3 ile 32 karakter olsun."; return; }
    if (p1.Length < 10 || p1.Length > 64) { mesaj = "Parola en az 10, en çok 64 karakter olsun."; return; }
    if (p1 != p2) { mesaj = "Parolalar aynı değil."; return; }
    try {
        EvseBulut.Depo.YoneticiYaz(kullanici, p1);
        bitti = true;
    } catch (Exception) {
        mesaj = "Parola kaydedilemedi. App_Data klasörüne yazma izni verin.";
    }
}
</script>
<% Response.Write(EvseBulut.Arayuz.Ust("Kurulum")); %>
<h1>İlk kurulum</h1>
<% if (bitti) { %>
<p class="iyi">Yönetici parolası kayıtlı. Bu sayfa artık parola değiştirmez. Parolayı unutursanız FTP ile <span class="gizli">App_Data/yonetici.json</span> dosyasını silin ve bu sayfayı yeniden açın.</p>
<p><a class="dugme" href="yonetim/giris.aspx">Yönetici girişi</a></p>
<% } else { %>
<p class="not">Siteyi herkese açmadan önce burada bir yönetici parolası belirleyin. Parola düz metin olarak saklanmaz.</p>
<% if (mesaj.Length > 0) { %><p class="hata"><%= EvseBulut.Arayuz.E(mesaj) %></p><% } %>
<form method="post" action="kurulum.aspx" autocomplete="off">
  <label for="kullanici">Kullanıcı adı</label>
  <input id="kullanici" name="kullanici" type="text" maxlength="32" value="admin" required>
  <label for="parola">Parola (en az 10 karakter)</label>
  <input id="parola" name="parola" type="password" minlength="10" maxlength="64" required autocomplete="new-password">
  <label for="parola2">Parola (tekrar)</label>
  <input id="parola2" name="parola2" type="password" minlength="10" maxlength="64" required autocomplete="new-password">
  <p><button type="submit">Kaydet</button></p>
</form>
<% } %>
<% Response.Write(EvseBulut.Arayuz.Alt()); %>
