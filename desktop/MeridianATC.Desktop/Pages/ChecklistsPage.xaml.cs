using System.Text.Json;
using System.Windows;
using System.Windows.Controls;

namespace MeridianATC.Desktop.Pages;

/// <summary>Checklists the engine knows, and the run controls.</summary>
public partial class ChecklistsPage : Page
{
    public ChecklistsPage()
    {
        InitializeComponent();
        Loaded += async (_, _) => await RefreshAsync();
    }

    private async Task RefreshAsync()
    {
        ChecklistList.Items.Clear();
        // The engine's state carries the book the current aeroplane loads.
        var s = await App.Engine.GetAsync("/api/state");
        if (s is { } st &&
            st.TryGetProperty("checklists", out var list) &&
            list.ValueKind == JsonValueKind.Array)
        {
            foreach (var item in list.EnumerateArray())
                if (item.ValueKind == JsonValueKind.String)
                    ChecklistList.Items.Add(item.GetString());
        }
        else
        {
            Status.Text = "no checklist book for this aeroplane yet";
        }
    }

    private async void Run_Click(object s, RoutedEventArgs e)
    {
        if (ChecklistList.SelectedItem is not string name) return;
        var r = await App.Engine.PostAsync("/api/checklist",
            new { name });
        Status.Text = r is { } res &&
            res.TryGetProperty("ok", out var ok) && ok.GetBoolean()
                ? $"running {name}" : $"could not run {name}";
    }

    private async void Refresh_Click(object s, RoutedEventArgs e)
        => await RefreshAsync();
}
