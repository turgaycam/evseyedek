<%@ Page Language="C#" CodePage="65001" %>
<script runat="server">
string mesaj = "";
void Page_Load() {
    EvseBulut.Ortam.Hazirla();
    if (!EvseBulut.Depo.Kuruldu()) { Response.Redirect("../kurulum.aspx"); return; }
    if (!string.Equals(Request.HttpMethod, "POST", StringComparison.OrdinalIgnoreCase)) return;
    int sn;
    if (EvseBulut.GirisKilit.Kilitli(out sn)) { mesaj = "Çok fazla deneme. " + sn + " saniye sonra yeniden deneyin."; return; }
    string kullanici = (Request.Form["kullanici"] ?? "").Trim();
    string parola = Request.Form["parola"] ?? "";
    if (EvseBulut.Depo.YoneticiGiris(kullanici, parola)) {
        EvseBulut.GirisKilit.Basarili();
        Session["yonetici"] = "1";
        EvseBulut.Arayuz.CsrfAl(Session);
        Response.Redirect("default.aspx");
        return;
    }
    EvseBulut.GirisKilit.Basarisiz();
    mesaj = "Kullanıcı adı veya parola yanlış.";
}
</script>
<% Response.Write(EvseBulut.Arayuz.Ust("Yönetici girişi")); %>
<h1>Yönetici girişi</h1>
<% if (mesaj.Length > 0) { %><p class="hata"><%= EvseBulut.Arayuz.E(mesaj) %></p><% } %>
<form method="post" action="giris.aspx" autocomplete="off">
  <label for="kullanici">Kullanıcı adı</label>
  <input id="kullanici" name="kullanici" type="text" maxlength="32" required autocomplete="username">
  <label for="parola">Parola</label>
  <input id="parola" name="parola" type="password" maxlength="64" required autocomplete="current-password">
  <p><button type="submit">Giriş</button></p>
</form>
<p><a href="../default.aspx">Herkese açık sayfa</a></p>
<% Response.Write(EvseBulut.Arayuz.Alt()); %>
