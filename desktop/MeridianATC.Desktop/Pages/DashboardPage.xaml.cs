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
            ApplySnapshot(s);
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
        if (EngineClient.StateFromEvent(e) is { } snapshot)
        {
            Dispatcher.Invoke(() => ApplySnapshot(snapshot));
            return;
        }
        // Only "event" envelopes carry a line worth logging; "state" and
        // "setup" payloads would draw a row of nothing every snapshot.
        if (!e.TryGetProperty("kind", out var kind)
            || kind.GetString() != "event")
            return;
        var type = e.TryGetProperty("type", out var ty)
            ? ty.GetString() : "";
        var text = e.TryGetProperty("text", out var t) ? t.GetString() : "";
        if (string.IsNullOrEmpty(text)) return;
        var speaker = e.TryGetProperty("speaker", out var sp)
            && sp.ValueKind == JsonValueKind.String
            && !string.IsNullOrEmpty(sp.GetString())
            ? sp.GetString() : type;
        Dispatcher.Invoke(() =>
        {
            Log.Items.Insert(0, $"{speaker}: {text}");
            if (Log.Items.Count > 100) Log.Items.RemoveAt(100);
        });
    }

    private void ApplySnapshot(JsonElement s)
    {
        Simulator.Text = EngineClient.SimulatorName(s);
        Callsign.Text = TryGet(s, "aircraft", "callsign") ?? "—";
        Phase.Text = TryGet(s, "flight", "phase") ?? "—";
        Frequency.Text = TryGet(s, "radio", "com1") ?? "—";
        AutonomousState.Text = TryGet(s, "autonomous", "enabled") == "yes" ? "on" : "off";
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

    private static string? TryGet(JsonElement e, params string[] path)
    {
        var node = e;
        foreach (var name in path)
            if (!node.TryGetProperty(name, out node))
                return null;
        return node.ValueKind switch
        {
            JsonValueKind.String => node.GetString(),
            JsonValueKind.Number => node.ToString(),
            JsonValueKind.True => "yes",
            JsonValueKind.False => "no",
            _ => null,
        };
    }
}
