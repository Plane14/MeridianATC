using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Flight monitor: the state snapshot and the traffic strips.</summary>
public partial class FlightMonitorPage : Page
{
    public FlightMonitorPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
    }

    private void Row(StackPanel where, string label, string value)
    {
        where.Children.Add(new TextBlock
        {
            Text = label, Foreground =
                new System.Windows.Media.SolidColorBrush(
                    System.Windows.Media.Color.FromRgb(0x8A, 0x95, 0xA5)),
        });
        where.Children.Add(new TextBlock
        {
            Text = value, FontSize = 15, FontWeight = FontWeights.SemiBold,
            Margin = new Thickness(0, 0, 0, 10),
        });
    }

    private async Task RefreshAsync()
    {
        Left.Children.Clear(); Middle.Children.Clear(); Right.Children.Clear();
        Traffic.Items.Clear();
        var s = await App.Engine.GetAsync("/api/state");
        if (s is not { } st) { Row(Left, "Engine", "no answer"); return; }

        if (st.TryGetProperty("aircraft", out var a))
        {
            Row(Left, "Callsign", Get(a, "callsign"));
            Row(Left, "Type", Get(a, "type"));
            Row(Left, "Wake", Get(a, "wake"));
        }
        if (st.TryGetProperty("position", out var p))
        {
            Row(Middle, "Altitude", $"{Get(p, "altitude_ft")} ft");
            Row(Middle, "Heading", $"{Get(p, "heading")}°");
            Row(Middle, "Ground speed", $"{Get(p, "ground_speed_kt")} kt");
            Row(Middle, "On ground", Get(p, "on_ground"));
            Row(Middle, "Squawk", Get(p, "squawk"));
        }
        if (st.TryGetProperty("flight", out var f))
        {
            Row(Right, "Phase", Get(f, "phase"));
            Row(Right, "Departure", Get(f, "departure"));
            Row(Right, "Destination", Get(f, "destination"));
            Row(Right, "Rules", Get(f, "rules"));
            Row(Right, "Runway", Get(f, "runway"));
        }
        if (st.TryGetProperty("traffic", out var t) &&
            t.ValueKind == JsonValueKind.Array)
            foreach (var strip in t.EnumerateArray())
                Traffic.Items.Add(
                    strip.ValueKind == JsonValueKind.String
                        ? strip.GetString()
                        : strip.ToString());
    }

    private async void Refresh_Click(object s, RoutedEventArgs e)
        => await RefreshAsync();

    private static string Get(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) ? v.ValueKind switch
        {
            JsonValueKind.String => v.GetString() ?? "",
            JsonValueKind.Number => v.ToString(),
            JsonValueKind.True => "yes",
            JsonValueKind.False => "no",
            _ => "",
        } : "";
}
