using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Diagnostics: the doctor's checks, plus the voice trainer.</summary>
public partial class DiagnosticsPage : Page
{
    public DiagnosticsPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshTrainerAsync();
    }

    private async void Doctor_Click(object s, RoutedEventArgs e)
    {
        DoctorList.Items.Clear();
        DoctorList.Items.Add("running…");
        var d = await App.Engine.GetAsync("/api/doctor");
        DoctorList.Items.Clear();
        if (d is not { } doc)
        { DoctorList.Items.Add("no answer"); return; }
        if (doc.TryGetProperty("checks", out var checks)
            && checks.ValueKind == JsonValueKind.Array)
            foreach (var c in checks.EnumerateArray())
            {
                var name = c.TryGetProperty("name", out var n)
                    ? n.GetString() : "?";
                // The report's verdict is a status word, not a boolean.
                var status = c.TryGetProperty("status", out var st)
                    ? st.GetString() : "?";
                var detail = c.TryGetProperty("detail", out var dt)
                    ? dt.GetString() : "";
                var fix = c.TryGetProperty("fix", out var f)
                    ? f.GetString() : "";
                DoctorList.Items.Add(
                    $"{status?.ToUpperInvariant()}  {name}  {detail}"
                    + (string.IsNullOrEmpty(fix) ? "" : $"  — {fix}"));
            }
        else
            DoctorList.Items.Add(doc.ToString());
    }

    private async Task RefreshTrainerAsync()
    {
        TrainerList.Items.Clear();
        var t = await App.Engine.GetAsync("/api/trainer");
        if (t is { } data)
        {
            if (data.TryGetProperty("keyterms", out var k)
                && k.ValueKind == JsonValueKind.Array)
                foreach (var term in k.EnumerateArray())
                    TrainerList.Items.Add($"keyterm: {term.GetString()}");
            if (data.TryGetProperty("corrections", out var c)
                && c.ValueKind == JsonValueKind.Object)
                foreach (var pair in c.EnumerateObject())
                    TrainerList.Items.Add(
                        $"correction: {pair.Name} → {pair.Value}");
        }
    }

    private async void AddKeyterm_Click(object s, RoutedEventArgs e)
    {
        var term = KeytermBox.Text.Trim();
        if (term.Length == 0) return;
        await App.Engine.PostAsync("/api/trainer",
            new { add_keyterms = new[] { term } });
        KeytermBox.Clear();
        await RefreshTrainerAsync();
    }

    private async void AddCorrection_Click(object s, RoutedEventArgs e)
    {
        var heard = HeardBox.Text.Trim();
        var meant = MeantBox.Text.Trim();
        if (heard.Length == 0 || meant.Length == 0) return;
        await App.Engine.PostAsync("/api/trainer",
            new { corrections = new Dictionary<string, string>
                  { [heard] = meant } });
        HeardBox.Clear(); MeantBox.Clear();
        await RefreshTrainerAsync();
    }
}
