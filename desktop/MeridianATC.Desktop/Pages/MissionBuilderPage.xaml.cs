using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Mission builder: continuous flight legs against the engine.</summary>
public partial class MissionBuilderPage : Page
{
    public MissionBuilderPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
    }

    private async Task RefreshAsync()
    {
        var s = await App.Engine.GetAsync("/api/mission");
        Legs.Items.Clear();
        if (s is { } st)
        {
            Status.Text = st.TryGetProperty("running", out var r)
                          && r.GetBoolean()
                ? $"running — {Get(st, "last_route")}"
                : "not running";
            if (st.TryGetProperty("legs", out var legs)
                && legs.ValueKind == JsonValueKind.Array)
                foreach (var leg in legs.EnumerateArray())
                    Legs.Items.Add(
                        $"Leg {Get(leg, "number")}: "
                        + $"{Get(leg, "departure")} → "
                        + $"{Get(leg, "destination")} "
                        + $"({Get(leg, "distance_nm")} NM)");
        }
    }

    private async void Start_Click(object s, RoutedEventArgs e)
    {
        var r = await App.Engine.PostAsync("/api/mission", new
        {
            action = "start",
            min_nm = double.TryParse(MinNm.Text, out var mn) ? mn : 30,
            max_nm = double.TryParse(MaxNm.Text, out var mx) ? mx : 150,
            service_minutes = int.TryParse(ServiceMin.Text, out var sv)
                ? sv : 30,
            reserve_factor = double.TryParse(Reserve.Text, out var rf)
                ? rf : 1.2,
        });
        Status.Text = r is { } res
            && res.TryGetProperty("ok", out var ok) && ok.GetBoolean()
                ? "mission started" : "refused — check the ranges";
    }

    private async void Next_Click(object s, RoutedEventArgs e)
    {
        var r = await App.Engine.PostAsync("/api/mission",
            new { action = "next" });
        await RefreshAsync();
    }

    private async void Stop_Click(object s, RoutedEventArgs e)
    {
        await App.Engine.PostAsync("/api/mission", new { action = "stop" });
        await RefreshAsync();
    }

    private static string Get(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) ? v.ValueKind switch
        {
            JsonValueKind.String => v.GetString() ?? "",
            JsonValueKind.Number => v.ToString(),
            _ => "",
        } : "";
}
