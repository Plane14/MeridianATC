using System.Text.Json;
using MeridianATC.Desktop;
using Xunit;

namespace MeridianATC.Desktop.Tests;

/// <summary>
/// The engine client's contract: endpoints parse, events surface, and a
/// dead engine never hangs a screen.
/// </summary>
public sealed class EngineClientTests
{
    [Theory]
    [InlineData("xplane", true, "X-Plane 12")]
    [InlineData("xplane", false, "X-Plane 12 (disconnected)")]
    [InlineData("simconnect", true, "Microsoft Flight Simulator")]
    [InlineData("manual", true, "Manual / replay")]
    public void SimulatorNamesComeFromState(string source, bool connected, string expected)
    {
        using var doc = JsonDocument.Parse(JsonSerializer.Serialize(new { source, connected }));
        Assert.Equal(expected, EngineClient.SimulatorName(doc.RootElement));
    }

    [Fact]
    public void StateEnvelopesUpdateTheDashboardRatherThanBeingIgnored()
    {
        using var message = JsonDocument.Parse("{\"kind\":\"state\",\"state\":{\"source\":\"xplane\",\"connected\":true}}");
        var state = EngineClient.StateFromEvent(message.RootElement);
        Assert.True(state.HasValue);
        Assert.Equal("X-Plane 12", EngineClient.SimulatorName(state.Value));
        using var line = JsonDocument.Parse("{\"kind\":\"event\",\"text\":\"radio\"}");
        Assert.Null(EngineClient.StateFromEvent(line.RootElement));
    }

    [Fact]
    public void MissingEngineDoesNotLaunchTheShellRecursively()
    {
        var directory = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString());
        Directory.CreateDirectory(directory);
        try
        {
            var shell = Path.Combine(directory, "MeridianATC.exe");
            File.WriteAllText(shell, "shell");
            Assert.Null(EngineClient.FindEngine(directory, shell));
            Directory.CreateDirectory(Path.Combine(directory, "engine"));
            var engine = Path.Combine(directory, "engine", "meridianatc.exe");
            File.WriteAllText(engine, "engine");
            Assert.Equal(engine, EngineClient.FindEngine(directory, shell));
        }
        finally { Directory.Delete(directory, true); }
    }

    [Fact]
    public void PortIsTheEngineOne()
    {
        Assert.Equal(8787, EngineClient.Port);
    }

    [Fact]
    public async Task GetOnNothingReturnsNullRatherThanThrowing()
    {
        // No engine running: the call must come back empty, not fault, so a
        // screen opened before the engine is up does not die with it.
        using var client = new EngineClient();
        var result = await client.GetAsync("/api/state",
            new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);
        Assert.Null(result);
    }

    [Fact]
    public async Task PostOnNothingReturnsNullRatherThanThrowing()
    {
        using var client = new EngineClient();
        var result = await client.PostAsync("/api/say", new { text = "hi" },
            new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);
        Assert.Null(result);
    }

    [Fact]
    public void EventPayloadParses()
    {
        // The shape the engine publishes on the socket, read the way the
        // dashboard reads it.
        using var doc = JsonDocument.Parse(
            "{\"kind\": \"controller\", \"text\": \"cleared to land\"," +
            "\"speaker\": \"Tower\"}");
        Assert.Equal("controller",
                     doc.RootElement.GetProperty("kind").GetString());
        Assert.Equal("cleared to land",
                     doc.RootElement.GetProperty("text").GetString());
    }
}
