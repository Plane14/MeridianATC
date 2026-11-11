using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using MeridianATC.Desktop.Pages;

namespace MeridianATC.Desktop;

/// <summary>The shell: navigation rail on the left, one screen on the right.</summary>
public partial class MainWindow : Window
{
    private static readonly (string Name, Func<Page> Make)[] Screens =
    {
        ("Dashboard", () => new DashboardPage()),
        ("Radio", () => new RadioPage()),
        ("Flight Plan", () => new FlightPlanPage()),
        ("Checklists", () => new ChecklistsPage()),
        ("Flight Monitor", () => new FlightMonitorPage()),
        ("Mission Builder", () => new MissionBuilderPage()),
        ("Transcript", () => new TranscriptPage()),
        ("Settings", () => new SettingsPage()),
        ("Diagnostics", () => new DiagnosticsPage()),
        ("First Run", () => new FirstRunPage()),
    };

    public MainWindow()
    {
        InitializeComponent();
        foreach (var (name, make) in Screens)
        {
            var button = new Button
            {
                Content = name,
                HorizontalContentAlignment = HorizontalAlignment.Left,
                Margin = new Thickness(0, 1, 0, 1),
                Padding = new Thickness(10, 7, 10, 7),
            };
            button.Click += (_, _) => ScreenFrame.Navigate(make());
            Nav.Children.Add(button);
        }
        ScreenFrame.Navigate(FirstRunDone()
            ? (Page)new DashboardPage()
            : new FirstRunPage());
        _ = ConnectAsync();
        InputBindings.Add(new KeyBinding(
            new RelayCommand(() => ScreenFrame.Navigate(new RadioPage())),
            Key.R, ModifierKeys.Control));
        InputBindings.Add(new KeyBinding(
            new RelayCommand(() => ScreenFrame.Navigate(new DashboardPage())),
            Key.D, ModifierKeys.Control));
        InputBindings.Add(new KeyBinding(
            new RelayCommand(() => ScreenFrame.Navigate(new TranscriptPage())),
            Key.T, ModifierKeys.Control));
    }

    private async Task ConnectAsync()
    {
        var ok = await App.Engine.ConnectAsync();
        Dispatcher.Invoke(() =>
            ConnectionLine.Text = ok
                ? "engine connected on 127.0.0.1:8787"
                : "engine not reachable");
    }

    /// <summary>Whether the shell's first-run page has been seen before.</summary>
    private static bool FirstRunDone()
    {
        var path = MarkerPath();
        if (File.Exists(path)) return true;
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path, DateTime.UtcNow.ToString("o"));
        }
        catch (IOException) { }
        return false;
    }

    private static string MarkerPath() => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "MeridianATC", "shell.firstrun");
}

/// <summary>An ICommand over a lambda, for the keyboard shortcuts.</summary>
public sealed class RelayCommand : ICommand
{
    private readonly Action _run;
    public RelayCommand(Action run) => _run = run;
    public bool CanExecute(object? p) => true;
    public void Execute(object? p) => _run();
    public event EventHandler? CanExecuteChanged
    { add { } remove { } }
}
