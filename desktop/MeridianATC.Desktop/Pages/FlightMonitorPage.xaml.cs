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

        // The surface advisor: where the aeroplane is on the taxi graph,
        // and what of the issued route is still ahead of it.
        Surface.Children.Clear();
        var surface = await App.Engine.GetAsync("/api/surface");
        if (surface is { } su && su.TryGetProperty("found", out var found)
            && found.GetBoolean())
        {
            Chip($"stand {Get(su, "stand")}", su, "stand");
            Chip($"taxiway {Get(su, "taxiway")}", su, "taxiway");
            if (su.TryGetProperty("next_taxiway", out _))
                Chip($"next: {Get(su, "next_taxiway")}", su, "next_taxiway");
            if (su.TryGetProperty("remaining_m", out var r)
                && r.ValueKind == JsonValueKind.Number)
                Chip($"{r.GetDouble():0} m to the limit", su, "remaining_m");
            if (su.TryGetProperty("clearance_limit", out _))
                Chip($"limit: {Get(su, "clearance_limit")}", su,
                     "clearance_limit");
        }
        else
        {
            Surface.Children.Add(new TextBlock
            {
                Text = "not on an airport the simulator has described",
                Foreground = new System.Windows.Media.SolidColorBrush(
                    System.Windows.Media.Color.FromRgb(0x8A, 0x95, 0xA5)),
            });
        }
    }

    private void Chip(string text, JsonElement root, string prop)
    {
        if (root.TryGetProperty(prop, out var v)
            && (v.ValueKind == JsonValueKind.String
                && string.IsNullOrEmpty(v.GetString())))
            return;
        Surface.Children.Add(new Border
        {
            Background = new System.Windows.Media.SolidColorBrush(
                System.Windows.Media.Color.FromRgb(0x1B, 0x22, 0x2D)),
            CornerRadius = new CornerRadius(10),
            Padding = new Thickness(10, 4, 10, 4),
            Margin = new Thickness(0, 0, 8, 4),
            Child = new TextBlock { Text = text, Foreground =
                new System.Windows.Media.SolidColorBrush(
                    System.Windows.Media.Color.FromRgb(0xDD, 0xE5, 0xEE)) },
        });
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
