using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Transcript/History: sessions on the left, their exchanges right.</summary>
public partial class TranscriptPage : Page
{
    private readonly List<int> _ids = new();

    public TranscriptPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
        App.Engine.OnEvent += OnEngineEvent;
        Unloaded += (_, _) => App.Engine.OnEvent -= OnEngineEvent;
    }

    private void OnEngineEvent(JsonElement e)
    {
        // Transcript lines are the radio's exchange: what the pilot said and
        // what a station answered, in the order they were said.
        if (!e.TryGetProperty("kind", out var k)
            || k.GetString() != "event"
            || !e.TryGetProperty("type", out var t))
            return;
        var type = t.GetString();
        if (type is not ("pilot" or "atc")) return;
        var who = e.TryGetProperty("speaker", out var sp)
            ? sp.GetString() : type;
        var text = e.TryGetProperty("text", out var x)
            ? x.GetString() : "";
        Dispatcher.Invoke(() => Lines.Items.Add($"{who}: {text}"));
    }

    private async Task RefreshAsync()
    {
        Sessions.Items.Clear();
        _ids.Clear();
        var live = await App.Engine.GetAsync("/api/transcript?limit=60");
        if (live is { } l &&
            l.TryGetProperty("transmissions", out var t)
            && t.ValueKind == JsonValueKind.Array)
        {
            Lines.Items.Clear();
            foreach (var row in t.EnumerateArray())
            {
                // Pilot rows carry no speaker name; the kind says who.
                var who = row.TryGetProperty("speaker", out var w)
                    && w.ValueKind == JsonValueKind.String
                    && !string.IsNullOrEmpty(w.GetString())
                    ? w.GetString()
                    : (row.TryGetProperty("kind", out var kd)
                        ? kd.GetString() : "");
                var text = row.TryGetProperty("text", out var x)
                    ? x.GetString() : "";
                Lines.Items.Add($"{who}: {text}");
            }
        }
        var h = await App.Engine.GetAsync("/api/history");
        if (h is { } data &&
            data.TryGetProperty("sessions", out var list)
            && list.ValueKind == JsonValueKind.Array)
        {
            foreach (var s in list.EnumerateArray())
            {
                var label = $"{Get(s, "callsign")}  "
                            + $"{Get(s, "departure")}→{Get(s, "destination")}";
                _ids.Add(s.GetProperty("id").GetInt32());
                Sessions.Items.Add(label);
            }
        }
    }

    private async void Sessions_Changed(object s,
        SelectionChangedEventArgs e)
    {
        var i = Sessions.SelectedIndex;
        if (i < 0 || i >= _ids.Count) return;
        var h = await App.Engine.GetAsync($"/api/history?session={_ids[i]}");
        Lines.Items.Clear();
        if (h is { } data &&
            data.TryGetProperty("transcript", out var list)
            && list.ValueKind == JsonValueKind.Array)
            foreach (var row in list.EnumerateArray())
                Lines.Items.Add($"{Get(row, "who")}: {Get(row, "text")}");
    }

    private async void Refresh_Click(object s, RoutedEventArgs e)
        => await RefreshAsync();

    private static string Get(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v)
        && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";
}
