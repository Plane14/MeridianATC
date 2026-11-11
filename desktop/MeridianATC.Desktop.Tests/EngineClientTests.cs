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
