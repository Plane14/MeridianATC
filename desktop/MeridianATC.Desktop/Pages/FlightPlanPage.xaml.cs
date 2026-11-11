using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Flight plan fields, written to the engine, plus SimBrief import.</summary>
public partial class FlightPlanPage : Page
{
    public FlightPlanPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
    }

    private async Task RefreshAsync()
    {
        var s = await App.Engine.GetAsync("/api/settings");
        if (s is { } data &&
            data.TryGetProperty("config", out var cfg) &&
            cfg.TryGetProperty("flight", out var f))
        {
            Callsign.Text = Get(f, "callsign");
            AircraftType.Text = Get(f, "aircraft_type");
            Departure.Text = Get(f, "departure");
            Destination.Text = Get(f, "destination");
            if (f.TryGetProperty("cruise_altitude_ft", out var c)
                && c.ValueKind == JsonValueKind.Number)
                Cruise.Text = c.GetInt32().ToString();
            if (cfg.TryGetProperty("simbrief", out var sb))
                SimbriefUser.Text = Get(sb, "username");
        }
    }

    private async void Apply_Click(object s, RoutedEventArgs e)
    {
        var body = new Dictionary<string, object?>
        {
            ["callsign"] = Callsign.Text.Trim(),
            ["aircraft_type"] = AircraftType.Text.Trim(),
            ["departure"] = Departure.Text.Trim(),
            ["destination"] = Destination.Text.Trim(),
        };
        if (int.TryParse(Cruise.Text, out var cruise))
            body["cruise_altitude_ft"] = cruise;
        var r = await App.Engine.PostAsync("/api/flight", body);
        Result.Text = r is { } res &&
            res.TryGetProperty("ok", out var ok) && ok.GetBoolean()
                ? "applied" : "refused";
    }

    private async void Simbrief_Click(object s, RoutedEventArgs e)
    {
        var r = await App.Engine.PostAsync("/api/simbrief",
            new { username = SimbriefUser.Text.Trim() });
        Result.Text = r is { } res &&
            res.TryGetProperty("ok", out var ok) && ok.GetBoolean()
                ? "SimBrief plan loaded"
                : "SimBrief import failed";
        await RefreshAsync();
    }

    private static string Get(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String
            ? v.GetString() ?? "" : "";
}
