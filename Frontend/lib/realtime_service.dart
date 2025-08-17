import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:http/http.dart' as http;

import 'config.dart';

class RealtimeService {
  RealtimeService({
    required this.userId,
    Duration? pollInterval,
    Duration? requestTimeout,
  }) : pollInterval = pollInterval ?? const Duration(seconds: 3),
       requestTimeout = requestTimeout ?? const Duration(seconds: 3);

  final String userId;
  final Duration pollInterval;
  final Duration requestTimeout;

  WebSocketChannel? _ws;
  Timer? _pollTimer;
  Timer? _reconnectTimer;
  Duration _backoff = const Duration(seconds: 1);

  final _ctrl = StreamController<Map<String, dynamic>>.broadcast();
  Stream<Map<String, dynamic>> get stream => _ctrl.stream;

  int? _lastTs;
  bool _gotWsMessage = false;
  bool _started = false;

  void connect() {
    if (_started) return;
    _started = true;
    _openWebSocket();
  }

  void _openWebSocket() {
    _cancelReconnect();
    _stopPolling();

    final url = wsUrlFor(userId);
    try {
      _ws = WebSocketChannel.connect(Uri.parse(url));
      _ws!.stream.listen(
        (raw) {
          _gotWsMessage = true;
          _backoff = const Duration(seconds: 1);
          _stopPolling();

          final map = _parseJson(raw);
          if (map != null && _shouldEmit(map)) _ctrl.add(map);
        },
        onError: (_) => _fallbackAndRetry(),
        onDone: _fallbackAndRetry,
        cancelOnError: true,
      );

      Future.delayed(const Duration(seconds: 5), () {
        if (!_gotWsMessage) _startPolling();
      });
    } catch (_) {
      _fallbackAndRetry();
    }
  }

  void _fallbackAndRetry() {
    _startPolling();
    _scheduleReconnect();
  }

  void _scheduleReconnect() {
    _cancelReconnect();
    _reconnectTimer = Timer(_backoff, _openWebSocket);
    final next = _backoff.inSeconds * 2;
    _backoff = Duration(seconds: next > 30 ? 30 : next);
  }

  void _cancelReconnect() {
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
  }

  void _startPolling() {
    if (_pollTimer != null) return;
    _pollTimer = Timer.periodic(pollInterval, (_) async {
      try {
        final r = await http
            .get(Uri.parse('$httpBase/last'))
            .timeout(requestTimeout);
        if (r.statusCode == 200) {
          final map = _parseJson(r.body);
          if (map != null && _shouldEmit(map)) _ctrl.add(map);
        }
      } catch (_) {}
    });
  }

  void _stopPolling() {
    _pollTimer?.cancel();
    _pollTimer = null;
  }

  Map<String, dynamic>? _parseJson(dynamic raw) {
    try {
      if (raw is Map<String, dynamic>) return raw;
      if (raw is String && raw.isNotEmpty) {
        final v = json.decode(raw);
        return v is Map<String, dynamic> ? v : null;
      }
    } catch (_) {}
    return null;
  }

  bool _shouldEmit(Map<String, dynamic> msg) {
    final ts = (msg['ts'] as num?)?.toInt();
    if (ts != null && ts == _lastTs) return false;
    _lastTs = ts ?? _lastTs;
    return true;
  }

  void dispose() {
    _cancelReconnect();
    _stopPolling();
    try {
      _ws?.sink.close();
    } catch (_) {}
    _ctrl.close();
    _started = false;
  }
}

final RealtimeService realtime = RealtimeService(userId: 'veronique');

Future<void> sendProfile(int age, String gender) async {
  try {
    final r = await http.post(
      Uri.parse('$httpBase/profile'),
      headers: {'Content-Type': 'application/json'},
      body: json.encode({"age": age, "sex": gender}),
    );
    if (r.statusCode == 200) {
      print("✅ Profile synced");
    } else {
      print("❌ Failed to sync profile: ${r.statusCode}");
    }
  } catch (e) {
    print("⚠️ Profile sync error: $e");
  }
}

Future<void> sendMood(String mood, {String? userId}) async {
  try {
    final uri = Uri.parse('$httpBase/profile');
    final body = <String, dynamic>{'mood': mood};
    if (userId != null && userId.isNotEmpty) body['userId'] = userId;

    final r = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: json.encode(body),
    );
    if (r.statusCode == 200) {
      print('✅ Mood sent: $mood');
    } else {
      print('❌ Failed to send mood (${r.statusCode}): ${r.body}');
    }
  } catch (e) {
    print('⚠️ Mood send error: $e');
  }
}
