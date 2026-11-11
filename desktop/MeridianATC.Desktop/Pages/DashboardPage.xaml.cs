using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Dashboard: state summary, the next-call hint, typed radio.</summary>
public partial class DashboardPage : Page
{
    public DashboardPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
        App.Engine.OnEvent += OnEngineEvent;
        Unloaded += (_, _) => App.Engine.OnEvent -= OnEngineEvent;
    }

    private async Task RefreshAsync()
    {
        var state = await App.Engine.GetAsync("/api/state");
        if (state is { } s)
        {
            Callsign.Text = TryGet(s, "callsign") ?? "—";
            Phase.Text = TryGet(s, "phase") ?? "—";
            Frequency.Text = TryGet(s, "com1_active") ?? "—";
        }
        var hint = await App.Engine.GetAsync("/api/guidance");
        if (hint is { } h)
        {
            HintTitle.Text = TryGet(h, "title") ?? "";
            HintDetail.Text = TryGet(h, "detail") ?? "";
            HintExample.Text = TryGet(h, "example") ?? "";
        }
    }

    private void OnEngineEvent(JsonElement e)
    {
        if (!e.TryGetProperty("kind", out var kind)) return;
        var text = e.TryGetProperty("text", out var t) ? t.GetString() : "";
        var speaker = e.TryGetProperty("speaker", out var sp)
            ? sp.GetString() : kind.GetString();
        Dispatcher.Invoke(() =>
        {
            Log.Items.Insert(0, $"{speaker}: {text}");
            if (Log.Items.Count > 100) Log.Items.RemoveAt(100);
        });
    }

    private async void Send_Click(object sender, RoutedEventArgs e)
    {
        var text = SayBox.Text.Trim();
        if (text.Length == 0) return;
        SayBox.Clear();
        await App.Engine.PostAsync("/api/say", new { text });
    }

    private async void Autonomous_Click(object sender, RoutedEventArgs e)
    {
        var enable = !AutonomousState.Text.Equals("on",
            StringComparison.OrdinalIgnoreCase);
        var r = await App.Engine.PostAsync("/api/autonomous",
            new { enabled = enable });
        if (r is { } res &&
            res.TryGetProperty("autonomous", out var a) &&
            a.TryGetProperty("enabled", out var on))
        {
            AutonomousState.Text = on.GetBoolean() ? "on" : "off";
        }
    }

    private static string? TryGet(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String
            ? v.GetString()
            : v.ValueKind is JsonValueKind.Number ? v.ToString() : null;
}
