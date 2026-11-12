using System.Diagnostics;
using System.IO;
using System.Net.Http;
using System.Net.Http.Json;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;

namespace MeridianATC.Desktop;

/// <summary>
/// Launches or attaches to the MeridianATC engine and speaks its local API.
/// </summary>
/// <remarks>
/// The engine is authoritative: every flight fact, clearance, and state the
/// shell shows comes from GET endpoints or the event socket. The shell never
/// computes aviation facts itself.
/// </remarks>
public sealed class EngineClient : IDisposable
{
    /// <summary>The loopback port the engine's FastAPI server listens on.</summary>
    public const int Port = 8787;

    private static readonly HttpClient Http = new()
    {
        BaseAddress = new Uri($"http://127.0.0.1:{Port}"),
        Timeout = TimeSpan.FromSeconds(10),
    };

    private Process? _engine;
    private ClientWebSocket? _socket;
    private CancellationTokenSource? _listen;
    private bool _disposed;

    /// <summary>Raised for every event the engine publishes on the socket.</summary>
    public event Action<JsonElement>? OnEvent;

    /// <summary>True once the engine answered a ping.</summary>
    public bool Connected { get; private set; }

    /// <summary>Start or attach to the engine, then begin the event stream.</summary>
    public async Task<bool> ConnectAsync(CancellationToken ct = default)
    {
        if (await PingAsync(ct))
        {
            Connected = true;
            _ = ListenAsync();
            return true;
        }
        if (await TryLaunchAsync(ct) && await PingAsync(ct, retries: 40,
                delay: TimeSpan.FromMilliseconds(500)))
        {
            Connected = true;
            _ = ListenAsync();
            return true;
        }
        Connected = false;
        return false;
    }

    private async Task<bool> PingAsync(CancellationToken ct, int retries = 1,
        TimeSpan? delay = null)
    {
        for (var i = 0; i < retries && !ct.IsCancellationRequested; i++)
        {
            try
            {
                var r = await Http.GetAsync("/api/state", ct);
                if (r.IsSuccessStatusCode) return true;
            }
            catch (HttpRequestException) { }
            catch (TaskCanceledException) { }
            if (delay is { } d) await Task.Delay(d, ct);
        }
        return false;
    }

    private async Task<bool> TryLaunchAsync(CancellationToken ct)
    {
        // The bundled engine beside the shell, then the development tree.
        var path = FindEngine(AppContext.BaseDirectory, Environment.ProcessPath);
        if (path is null) return false;
        try
        {
            _engine = Process.Start(new ProcessStartInfo(path,
                "gui --no-browser")
            {
                UseShellExecute = false,
                CreateNoWindow = true,
            });
            return _engine is not null;
        }
        catch (Exception)
        {
            return false;
        }
    }

    public static string? FindEngine(string directory, string? processPath)
    {
        var candidates = new[]
        {
            Path.Combine(directory, "engine", "meridianatc.exe"),
            Path.Combine(directory, "MeridianATC.exe"),
        };
        return candidates.FirstOrDefault(p => File.Exists(p) &&
            !string.Equals(Path.GetFullPath(p), processPath,
                StringComparison.OrdinalIgnoreCase));
    }

    public static string SimulatorName(JsonElement state)
    {
        var source = state.TryGetProperty("source", out var s) ? s.GetString() : "";
        var name = source switch
        {
            "xplane" => "X-Plane 12",
            "simconnect" => "Microsoft Flight Simulator",
            _ => "Manual / replay",
        };
        return state.TryGetProperty("connected", out var c) && c.ValueKind == JsonValueKind.True
            ? name : name + " (disconnected)";
    }

    public static JsonElement? StateFromEvent(JsonElement envelope)
    {
        return envelope.TryGetProperty("kind", out var kind) && kind.GetString() == "state"
            && envelope.TryGetProperty("state", out var state) ? state.Clone() : null;
    }

    /// <summary>GET an endpoint and read its JSON body.</summary>
    public async Task<JsonElement?> GetAsync(string path,
        CancellationToken ct = default)
    {
        try
        {
            using var r = await Http.GetAsync(path, ct);
            if (!r.IsSuccessStatusCode) return null;
            return JsonDocument.Parse(await r.Content.ReadAsStringAsync(ct))
                .RootElement.Clone();
        }
        catch (Exception) { return null; }
    }

    /// <summary>POST a JSON body and read the answer.</summary>
    public async Task<JsonElement?> PostAsync(string path, object body,
        CancellationToken ct = default)
    {
        try
        {
            using var r = await Http.PostAsJsonAsync(path, body, ct);
            if (!r.IsSuccessStatusCode) return null;
            return JsonDocument.Parse(await r.Content.ReadAsStringAsync(ct))
                .RootElement.Clone();
        }
        catch (Exception) { return null; }
    }

    private async Task ListenAsync()
    {
        _listen?.Cancel();
        _listen = new CancellationTokenSource();
        var token = _listen.Token;
        while (!token.IsCancellationRequested)
        {
            try
            {
                _socket?.Dispose();
                _socket = new ClientWebSocket();
                await _socket.ConnectAsync(
                    new Uri($"ws://127.0.0.1:{Port}/ws"), token);
                var buffer = new byte[64 * 1024];
                while (_socket.State == WebSocketState.Open &&
                       !token.IsCancellationRequested)
                {
                    var message = new MemoryStream();
                    WebSocketReceiveResult chunk;
                    do
                    {
                        chunk = await _socket.ReceiveAsync(buffer, token);
                        if (chunk.MessageType == WebSocketMessageType.Close)
                            break;
                        message.Write(buffer, 0, chunk.Count);
                    } while (!chunk.EndOfMessage);
                    if (chunk.MessageType == WebSocketMessageType.Close) break;
                    try
                    {
                        using var doc = JsonDocument.Parse(
                            message.ToArray());
                        OnEvent?.Invoke(doc.RootElement.Clone());
                    }
                    catch (JsonException) { }
                }
            }
            catch (Exception)
            {
                // Reconnect: the engine may still be starting.
            }
            try { await Task.Delay(2000, token); }
            catch (TaskCanceledException) { break; }
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _listen?.Cancel();
        _socket?.Dispose();
        // The engine belongs to the user: started by us, it may outlive the
        // window the way a co-pilot outlives the briefing.
    }
}
