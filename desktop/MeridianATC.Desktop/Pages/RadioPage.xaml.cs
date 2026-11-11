using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace MeridianATC.Desktop.Pages;

/// <summary>Radio: tune COM1, set the squawk, pick a station in range.</summary>
public partial class RadioPage : Page
{
    private readonly List<(string Label, double Mhz)> _stations = new();

    public RadioPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
    }

    private async Task RefreshAsync()
    {
        var data = await App.Engine.GetAsync("/api/frequencies");
        Stations.Items.Clear();
        _stations.Clear();
        if (data is { } d &&
            d.TryGetProperty("stations", out var list) &&
            list.ValueKind == JsonValueKind.Array)
        {
            foreach (var st in list.EnumerateArray())
            {
                var callsign = st.TryGetProperty("callsign", out var c)
                    ? c.GetString() : "?";
                var mhz = st.TryGetProperty("mhz", out var m)
                    ? m.GetDouble() : 0.0;
                var dist = st.TryGetProperty("distance_nm", out var dn)
                    ? dn.GetDouble() : 0.0;
                var label = $"{callsign}  {mhz:0.000}  ({dist:F0} NM)";
                _stations.Add((label, mhz));
                Stations.Items.Add(label);
            }
        }
    }

    private async void Tune_Click(object s, RoutedEventArgs e)
        => await TuneAsync(standby: false);
    private async void Standby_Click(object s, RoutedEventArgs e)
        => await TuneAsync(standby: true);

    private async Task TuneAsync(bool standby)
    {
        if (!double.TryParse(FreqBox.Text, out var mhz))
        {
            TuneResult.Text = "Enter a frequency like 118.310";
            return;
        }
        var r = await App.Engine.PostAsync("/api/tune",
            new { mhz, standby });
        TuneResult.Text = r is { } res
            && res.TryGetProperty("ok", out var ok) && ok.GetBoolean()
                ? $"tuned {mhz:0.000}" +
                  (res.TryGetProperty("station", out var st)
                      && st.GetString() is { Length: > 0 } name
                      ? $" — {name}" : "")
                : "tune refused";
    }

    private async void Swap_Click(object s, RoutedEventArgs e)
        => await App.Engine.PostAsync("/api/swap", new { radio = 1 });

    private async void Squawk_Click(object s, RoutedEventArgs e)
        => await App.Engine.PostAsync("/api/squawk",
            new { code = SquawkBox.Text.Trim() });

    private async void Stations_DoubleClick(object s, MouseButtonEventArgs e)
    {
        if (Stations.SelectedIndex < 0 ||
            Stations.SelectedIndex >= _stations.Count) return;
        FreqBox.Text = _stations[Stations.SelectedIndex].Mhz.ToString("0.000");
        await TuneAsync(standby: false);
    }
}
