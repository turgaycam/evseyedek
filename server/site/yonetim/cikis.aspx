<%@ Page Language="C#" CodePage="65001" %>
<script runat="server">
void Page_Load() {
    Session.Clear();
    Session.Abandon();
    Response.Redirect("giris.aspx");
}
</script>
