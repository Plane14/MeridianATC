using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Settings: the AI/speech/offline/consent cluster, through the API.</summary>
public partial class SettingsPage : Page
{
    public SettingsPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
    }

    private async Task RefreshAsync()
    {
        var s = await App.Engine.GetAsync("/api/settings");
        if (s is not { } data) return;
        if (data.TryGetProperty("config", out var cfg))
        {
            if (cfg.TryGetProperty("ai", out var ai))
            {
                Select(Provider, Get(ai, "provider"));
                Model.Text = Get(ai, "model");
                Phrasing.IsChecked = ai.TryGetProperty("phrasing", out var p)
                    && p.ValueKind == JsonValueKind.True;
            }
            if (cfg.TryGetProperty("speech", out var sp))
            {
                Select(SttEngine, Get(sp, "stt_engine"));
                DeepgramModel.Text = Get(sp, "deepgram_model");
            }
            if (cfg.TryGetProperty("network", out var net))
            {
                Offline.IsChecked = net.TryGetProperty("offline", out var o)
                    && o.ValueKind == JsonValueKind.True;
                Consent.IsChecked = net.TryGetProperty("cloud_consent",
                    out var c) && c.ValueKind == JsonValueKind.True;
            }
        }
    }

    private async void Save_Click(object s, RoutedEventArgs e)
    {
        var changes = new Dictionary<string, object?>
        {
            ["ai.provider"] = (Provider.SelectedItem as ComboBoxItem)
                ?.Content?.ToString(),
            ["ai.model"] = Model.Text.Trim(),
            ["ai.phrasing"] = Phrasing.IsChecked == true,
            ["speech.stt_engine"] = (SttEngine.SelectedItem as ComboBoxItem)
                ?.Content?.ToString(),
            ["speech.deepgram_model"] = DeepgramModel.Text.Trim(),
            ["network.offline"] = Offline.IsChecked == true,
            ["network.cloud_consent"] = Consent.IsChecked == true,
        };
        // Secrets go only when the pilot typed one: empty keeps what is set.
        if (ApiKey.Password.Length > 0)
            changes["ai.api_key"] = ApiKey.Password;
        if (DeepgramKey.Password.Length > 0)
            changes["speech.deepgram_key"] = DeepgramKey.Password;
        var r = await App.Engine.PostAsync("/api/settings", changes);
        Result.Text = r is { } res &&
            res.TryGetProperty("ok", out var ok) && ok.GetBoolean()
                ? "saved" : "refused";
    }

    private static void Select(ComboBox box, string value)
    {
        foreach (var item in box.Items)
            if (item is ComboBoxItem c &&
                c.Content?.ToString() == value)
            { box.SelectedItem = item; return; }
    }

    private static string Get(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v)
        && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";
}
