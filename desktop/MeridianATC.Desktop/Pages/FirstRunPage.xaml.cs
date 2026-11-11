using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>First run: the consent StormATC asked before any cloud use.</summary>
public partial class FirstRunPage : Page
{
    public FirstRunPage()
    {
        InitializeComponent();
        Loaded += async (_, _) =>
        {
            var s = await App.Engine.GetAsync("/api/settings");
            if (s is { } d &&
                d.TryGetProperty("config", out var cfg) &&
                cfg.TryGetProperty("network", out var net) &&
                net.TryGetProperty("cloud_consent", out var c))
                Consent.IsChecked = c.ValueKind == JsonValueKind.True;
        };
    }

    private async void Continue_Click(object s, RoutedEventArgs e)
    {
        var r = await App.Engine.PostAsync("/api/settings",
            new Dictionary<string, object?>
            { ["network.cloud_consent"] = Consent.IsChecked == true });
        Result.Text = Consent.IsChecked == true
            ? "Consent given — cloud services are available."
            : "No consent — the program runs fully offline.";
    }
}
