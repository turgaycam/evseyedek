<%@ Application Language="C#" %>
<script runat="server">
void Application_Start(object sender, EventArgs e) {
    try {
        EvseBulut.Ortam.Hazirla();
    } catch (Exception) {
        // Ilk istekte yeniden denenir. Kurulum kilavuzu App_Data yazma iznini anlatir.
    }
}
</script>
