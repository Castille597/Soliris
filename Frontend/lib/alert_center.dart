import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

class AppAlert {
  AppAlert({
    required this.id,
    required this.title,
    required this.body,
    required this.level,
    required this.ts,
    this.unread = true,
  });

  final String id;
  final String title;
  final String body;
  final String level;
  final DateTime ts;
  bool unread;

  Map<String, dynamic> toJson() => {
    'id': id,
    'title': title,
    'body': body,
    'level': level,
    'ts': ts.millisecondsSinceEpoch,
    'unread': unread,
  };

  static AppAlert fromJson(Map<String, dynamic> j) => AppAlert(
    id: j['id'] as String,
    title: j['title'] as String,
    body: j['body'] as String,
    level: j['level'] as String,
    ts: DateTime.fromMillisecondsSinceEpoch((j['ts'] as num).toInt()),
    unread: (j['unread'] as bool?) ?? false,
  );
}

class AlertCenter {
  AlertCenter._();
  static final instance = AlertCenter._();

  final ValueNotifier<int> unread = ValueNotifier<int>(0);
  final ValueNotifier<List<AppAlert>> alertsListenable =
      ValueNotifier<List<AppAlert>>(<AppAlert>[]);

  final List<AppAlert> _inbox = <AppAlert>[];
  final Map<String, DateTime> _cooldown = {};
  StreamSubscription? _sub;

  static const _storageKey = 'alert_history_v1';
  static const int _maxItems = 150;

  String _lastRecKey = '';

  Future<void> init() async {
    try {
      final sp = await SharedPreferences.getInstance();
      final raw = sp.getString(_storageKey);
      if (raw != null && raw.isNotEmpty) {
        final List list = jsonDecode(raw);
        _inbox
          ..clear()
          ..addAll(
            list
                .map((e) => AppAlert.fromJson(Map<String, dynamic>.from(e)))
                .toList()
                .cast<AppAlert>(),
          );
        _notify();
      }
    } catch (_) {
      /* ignore */
    }
  }

  void bindTo(Stream<dynamic> events) {
    _sub?.cancel();
    _sub = events.listen(_onEvent, onError: (_) {});
  }

  void dispose() => _sub?.cancel();

  Map<String, dynamic> _asMap(dynamic v) {
    if (v is Map) return v.map((k, val) => MapEntry(k.toString(), val));
    if (v is String) {
      try {
        final d = jsonDecode(v);
        if (d is Map) return d.map((k, val) => MapEntry(k.toString(), val));
      } catch (_) {}
    }
    return const {};
  }

  num? _num(dynamic v) {
    if (v is num) return v;
    if (v is String) return num.tryParse(v);
    return null;
  }

  String _str(dynamic v) => v == null ? '' : v.toString();

  int? _epochMs(dynamic ts) {
    final n = _num(ts);
    if (n == null) return null;
    final i = n.toInt();
    return i < 1000000000000 ? i * 1000 : i;
  }

  num? _extractFromText(String text, RegExp re) {
    final m = re.firstMatch(text);
    if (m == null) return null;
    return num.tryParse(m.group(1)!.replaceAll(',', '.'));
  }

  void _onEvent(dynamic ev) {
    final m = _asMap(ev);

    final List rawAlerts = (m['alerts'] as List?) ?? const [];
    for (final a in rawAlerts) {
      final am = _asMap(a);
      _add(
        AppAlert(
          id: _str(am['id']).isNotEmpty
              ? _str(am['id'])
              : 'srv-${DateTime.now().millisecondsSinceEpoch}',
          title: _str(am['title']).isEmpty ? 'Alert' : _str(am['title']),
          body: _str(am['body']),
          level: _str(am['level']).isEmpty ? 'warn' : _str(am['level']),
          ts: DateTime.fromMillisecondsSinceEpoch(
            _epochMs(am['ts']) ?? DateTime.now().millisecondsSinceEpoch,
          ),
        ),
      );
    }

    final tel = _asMap(m['telemetry']);
    final hr = _num(tel['hr'])?.toInt();
    final spo2 = _num(tel['spo2'])?.toInt();
    final tskin = _num(tel['temp_skin'])?.toDouble();

    if (hr != null && hr >= 105) {
      _once(
        key: 'hr_high',
        title: 'Elevated heart rate',
        body: 'Heart rate is $hr BPM.',
        level: 'warn',
        ts: tel['ts'],
        cooldownMin: 10,
      );
    }
    if (spo2 != null && spo2 <= 92) {
      _once(
        key: 'spo2_low',
        title: 'Low SpO₂',
        body: 'Blood oxygen is $spo2%.',
        level: 'alert',
        ts: tel['ts'],
        cooldownMin: 30,
      );
    }
    if (tskin != null && tskin >= 38.0) {
      _once(
        key: 'skin_temp_high',
        title: 'High skin temperature',
        body: 'Skin temperature is ${tskin.toStringAsFixed(1)}°C.',
        level: 'warn',
        ts: tel['ts'],
        cooldownMin: 15,
      );
    }

    _snapshotRecommendation(m);

    _notify();
  }

  void _snapshotRecommendation(Map<String, dynamic> m) {
    final rec = _asMap(m['recommendation']);
    if (rec.isEmpty) return;

    final head = _str(rec['headline']).trim();
    final expl = _str(rec['explanation']).trim();
    final risk = _str(rec['risk']).toLowerCase();
    final key = '$risk|$head|$expl';

    if (key == _lastRecKey) return;
    _lastRecKey = key;

    final tsMs = _epochMs(m['ts']) ?? DateTime.now().millisecondsSinceEpoch;

    _add(
      AppAlert(
        id: 'rec-$tsMs',
        title: head.isEmpty ? 'Recommendation' : head,
        body: expl.isEmpty ? 'Updated guidance' : expl,
        level: (risk == 'high')
            ? 'alert'
            : (risk == 'medium' || risk == 'moderate')
            ? 'warn'
            : 'info',
        ts: DateTime.fromMillisecondsSinceEpoch(tsMs),
      ),
    );
  }

  void _once({
    required String key,
    required String title,
    required String body,
    required String level,
    required dynamic ts,
    int cooldownMin = 10,
  }) {
    final now = DateTime.now();
    final last = _cooldown[key];
    if (last != null && now.difference(last).inMinutes < cooldownMin) return;
    _cooldown[key] = now;
    _add(
      AppAlert(
        id: '$key-${now.millisecondsSinceEpoch}',
        title: title,
        body: body,
        level: level,
        ts: DateTime.fromMillisecondsSinceEpoch(
          _epochMs(ts) ?? now.millisecondsSinceEpoch,
        ),
      ),
    );
  }

  void _add(AppAlert a) {
    _inbox.insert(0, a);
    if (_inbox.length > _maxItems) {
      _inbox.removeRange(_maxItems, _inbox.length);
    }
    _save();
  }

  void markAllRead() {
    for (final a in _inbox) a.unread = false;
    _notify();
    _save();
  }

  void _notify() {
    unread.value = _inbox.where((a) => a.unread).length;
    alertsListenable.value = List<AppAlert>.unmodifiable(_inbox);
  }

  Future<void> _save() async {
    try {
      final sp = await SharedPreferences.getInstance();
      final list = _inbox.map((e) => e.toJson()).toList();
      await sp.setString(_storageKey, jsonEncode(list));
    } catch (_) {
      /* ignore */
    }
  }
}
