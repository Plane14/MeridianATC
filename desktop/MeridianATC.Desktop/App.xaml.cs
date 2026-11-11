using System.Windows;

namespace MeridianATC.Desktop;

/// <summary>Application entry: starts the engine client, shows the shell.</summary>
public partial class App : Application
{
    /// <summary>The shared engine connection for every screen.</summary>
    public static EngineClient Engine { get; } = new();

    protected override void OnExit(ExitEventArgs e)
    {
        Engine.Dispose();
        base.OnExit(e);
    }
}
