import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';

import '../widgets/health_metric_card.dart';
import '../widgets/multi_metric_chart.dart';
import '../widgets/app_top_bar.dart';

import '../realtime_service.dart';
import '../alert_center.dart';
import 'health_dashboard_pages.dart';

class DocumentationPage extends StatefulWidget {
  const DocumentationPage({super.key});

  @override
  State<DocumentationPage> createState() => _DocumentationPageState();
}

class _DocumentationPageState extends State<DocumentationPage> {
  late final RealtimeService _rt;
  StreamSubscription<Map<String, dynamic>>? _sub;

  static bool _alertsBound = false;

  final Map<String, List<FlSpot>> _series = <String, List<FlSpot>>{
    'HR': <FlSpot>[],
    'SpO₂': <FlSpot>[],
    'Skin temp': <FlSpot>[],
    'Resp rate': <FlSpot>[],
    'Steps/min': <FlSpot>[],
  };

  final Map<String, bool> _visible = <String, bool>{
    'HR': true,
    'SpO₂': true,
    'Skin temp': false,
    'Resp rate': false,
    'Steps/min': false,
  };

  Map<String, dynamic>? _lastMsg;
  DateTime _currentDay = DateTime.now();

  double? _lastStepsCum;
  double? _lastXForSteps;

  double _lpPitch = 0, _lpRoll = 0, _lpMag = 1.0;
  double _ema(double prev, double next, [double a = 0.25]) =>
      prev + a * (next - prev);

  Map<String, dynamic> _asMap(dynamic v) {
    if (v is Map) return v.map((k, val) => MapEntry(k.toString(), val));
    return const {};
  }

  num? _num(dynamic v) {
    if (v is num) return v;
    if (v is String) return num.tryParse(v);
    return null;
  }

  String _fmtNum(num? v, {String unit = ''}) {
    if (v == null) return '--';
    final isInt = v is int || v == v.roundToDouble();
    return '${isInt ? v.toInt() : v.toStringAsFixed(1)}$unit';
  }

  String? _mapPosture(dynamic v) {
    final s = v?.toString().toLowerCase().trim();
    switch (s) {
      case 'standing':
      case 'stand':
        return 'Standing';
      case 'sitting':
      case 'sit':
        return 'Sitting';
      case 'lying':
      case 'lie':
      case 'supine':
        return 'Lying';
      case 'walking':
      case 'walk':
        return 'Walking';
      case 'running':
      case 'run':
        return 'Running';
      case 'fall':
        return 'Fall';
    }
    return null;
  }

  Map<String, num?> _readIMU(Map tel) {
    final m = (tel['m'] as Map?) ?? tel;
    return <String, num?>{
      'ax': (m['ax'] as num?),
      'ay': (m['ay'] as num?),
      'az': (m['az'] as num?),
    };
  }

  String _derivePosture(Map tel) {
    final imu = _readIMU(tel);
    final ax = (imu['ax'] ?? 0).toDouble();
    final ay = (imu['ay'] ?? 0).toDouble();
    final az = (imu['az'] ?? 1).toDouble();

    final pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / pi;
    final roll = atan2(ay, az) * 180 / pi;

    _lpPitch = _ema(_lpPitch, pitch);
    _lpRoll = _ema(_lpRoll, roll);

    if (_lpPitch.abs() > 45) return 'Lying';
    if (_lpPitch.abs() < 20 && _lpRoll.abs() < 20) return 'Standing';
    if (_lpPitch.abs() < 20) return 'Sitting';
    return '—';
  }

  String _deriveActivity(Map tel) {
    final imu = _readIMU(tel);
    final ax = (imu['ax'] ?? 0).toDouble();
    final ay = (imu['ay'] ?? 0).toDouble();
    final az = (imu['az'] ?? 1).toDouble();

    final mag = sqrt(ax * ax + ay * ay + az * az);
    _lpMag = _ema(_lpMag, mag);

    final dev = (_lpMag - 1.0).abs();
    if (dev < 0.05) return 'Still';
    if (dev < 0.25) return 'Light';
    return 'Active';
  }

  void _addPoint(String key, double x, double? y) {
    if (y == null) return;
    final list = _series[key] ??= <FlSpot>[];
    list.add(FlSpot(x, y));
    list.removeWhere((p) => p.x < 0 || p.x > 1440);
    list.sort((a, b) => a.x.compareTo(b.x));
    if (list.length > 2000) list.removeRange(0, list.length - 2000);
  }

  void _maybeResetAtMidnight() {
    final now = DateTime.now();
    if (now.year != _currentDay.year ||
        now.month != _currentDay.month ||
        now.day != _currentDay.day) {
      for (final k in _series.keys) {
        _series[k] = <FlSpot>[];
      }
      _currentDay = now;
      _lastStepsCum = null;
      _lastXForSteps = null;
    }
  }

  double _minCardWidth(BuildContext context) {
    final scale = MediaQuery.of(context).textScaleFactor.clamp(0.9, 1.6);
    return 160.0 * scale;
  }

  @override
  void initState() {
    super.initState();

    _rt = realtime;
    _rt.connect();

    if (!_alertsBound) {
      AlertCenter.instance.bindTo(_rt.stream);
      _alertsBound = true;
    }

    _sub = _rt.stream.listen((msg) {
      _lastMsg = msg;
      _maybeResetAtMidnight();

      final root = _asMap(msg);
      final tel = _asMap(root['telemetry']);
      final Map<String, dynamic> base = {...root, ...tel};
      final Map<String, dynamic> m = {
        ..._asMap(root['m']),
        ..._asMap(tel['m']),
      };

      final tsRaw =
          base['timestamp'] ?? base['ts'] ?? base['time'] ?? DateTime.now();
      final ts = MultiMetricChart.parseTimestamp(tsRaw);
      final x = MultiMetricChart.minutesSinceMidnight(ts);

      final hr = _num(base['hr'])?.toDouble();
      final spo2 = _num(base['spo2'])?.toDouble();
      final skin = _num(base['temp_skin'])?.toDouble();
      final respRate =
          (_num(base['rr']) ?? _num(base['resp']) ?? _num(base['resp_rate']))
              ?.toDouble();

      final stepsCum =
          (_num(base['steps_today']) ??
                  _num(base['steps_total']) ??
                  _num(base['steps']) ??
                  _num(m['steps']))
              ?.toDouble();

      setState(() {
        _addPoint('HR', x, hr);
        _addPoint('SpO₂', x, spo2);
        _addPoint('Skin temp', x, skin);
        _addPoint('Resp rate', x, respRate);

        if (stepsCum != null) {
          if (_lastStepsCum != null && _lastXForSteps != null) {
            final dtMin = x - _lastXForSteps!;
            if (dtMin > 0) {
              final dSteps = (stepsCum - _lastStepsCum!).clamp(
                0.0,
                double.infinity,
              );
              final rate = dSteps / dtMin;
              _addPoint('Steps/min', x, rate);
            }
          }
          _lastStepsCum = stepsCum;
          _lastXForSteps = x;
        }
      });
    });
  }

  @override
  void dispose() {
    _sub?.cancel();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final today = DateTime.now().toLocal().toString().split(' ').first;
    final double cardW = _minCardWidth(context);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      body: SafeArea(
        child: StreamBuilder<Map<String, dynamic>>(
          stream: _rt.stream,
          initialData: _lastMsg,
          builder: (context, snap) {
            final msg = snap.data ?? _lastMsg ?? const <String, dynamic>{};

            final root = _asMap(msg);
            final tel = _asMap(root['telemetry']);
            final Map<String, dynamic> base = {...root, ...tel};
            final Map<String, dynamic> m = {
              ..._asMap(root['m']),
              ..._asMap(tel['m']),
            };

            final hrTxt = _fmtNum(_num(base['hr']), unit: ' BPM');
            final spo2Txt = _fmtNum(_num(base['spo2']), unit: '%');
            final skinTxt = _fmtNum(_num(base['temp_skin']), unit: '°C');

            final stepsAny =
                (_num(base['steps_today']) ??
                _num(base['steps_total']) ??
                _num(base['steps']) ??
                _num(m['steps']));
            final stepsTxt = _fmtNum(stepsAny, unit: '');

            String? postureFromBackend =
                _mapPosture(base['posture']) ??
                _mapPosture(base['activity']) ??
                _mapPosture(m['posture']) ??
                _mapPosture(m['activity']);

            postureFromBackend ??= (() {
              if ((_num(base['lying']) ?? _num(m['lying'])) == 1)
                return 'Lying';
              if ((_num(base['sitting']) ?? _num(m['sitting'])) == 1) {
                return 'Sitting';
              }
              if ((_num(base['standing']) ?? _num(m['standing'])) == 1) {
                return 'Standing';
              }
              return null;
            })();

            final posture = postureFromBackend ?? _derivePosture(base);

            final actRaw = (base['activity'] ?? m['activity'])?.toString();
            final activity = (actRaw != null && actRaw.isNotEmpty)
                ? actRaw[0].toUpperCase() + actRaw.substring(1)
                : _deriveActivity(base);

            return SingleChildScrollView(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const AppTopBar(),
                  const SizedBox(height: 20),

                  Center(
                    child: Text(
                      'Health Metrics',
                      style: TextStyle(
                        fontSize: 22,
                        fontWeight: FontWeight.bold,
                        color: theme.textTheme.bodyMedium?.color,
                      ),
                    ),
                  ),

                  const SizedBox(height: 24),

                  SingleChildScrollView(
                    scrollDirection: Axis.horizontal,
                    child: Row(
                      children: [
                        ConstrainedBox(
                          constraints: BoxConstraints(minWidth: cardW),
                          child: HealthMetricCard(
                            title: 'Heart rate',
                            value: hrTxt,
                            icon: Icons.favorite,
                          ),
                        ),
                        const SizedBox(width: 12),
                        ConstrainedBox(
                          constraints: BoxConstraints(minWidth: cardW),
                          child: HealthMetricCard(
                            title: 'SpO₂',
                            value: spo2Txt,
                            icon: Icons.water_drop,
                          ),
                        ),
                        const SizedBox(width: 12),
                        ConstrainedBox(
                          constraints: BoxConstraints(minWidth: cardW),
                          child: HealthMetricCard(
                            title: 'Skin temp',
                            value: skinTxt,
                            icon: Icons.thermostat,
                          ),
                        ),
                        const SizedBox(width: 12),
                        ConstrainedBox(
                          constraints: BoxConstraints(minWidth: cardW),
                          child: HealthMetricCard(
                            title: 'Steps',
                            value: stepsTxt,
                            icon: Icons.directions_walk,
                          ),
                        ),
                        const SizedBox(width: 12),
                        ConstrainedBox(
                          constraints: BoxConstraints(minWidth: cardW),
                          child: HealthMetricCard(
                            title: 'Posture',
                            value: posture,
                            icon: Icons.accessibility_new,
                          ),
                        ),
                        const SizedBox(width: 12),
                        ConstrainedBox(
                          constraints: BoxConstraints(minWidth: cardW),
                          child: HealthMetricCard(
                            title: 'Activity',
                            value: activity,
                            icon: Icons.directions_run,
                          ),
                        ),
                      ],
                    ),
                  ),

                  const SizedBox(height: 24),

                  Center(
                    child: ElevatedButton.icon(
                      icon: const Icon(Icons.history),
                      label: const Text(
                        'View History',
                        style: TextStyle(fontWeight: FontWeight.w800),
                      ),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.orange,
                        foregroundColor: Colors.white,
                        padding: const EdgeInsets.symmetric(
                          horizontal: 22,
                          vertical: 14,
                        ),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(20),
                        ),
                        elevation: 0,
                      ),
                      onPressed: () {
                        Navigator.of(context).push(
                          MaterialPageRoute(
                            builder: (_) => const HealthDashboardPage(),
                          ),
                        );
                      },
                    ),
                  ),

                  const SizedBox(height: 12),

                  Row(
                    children: [
                      _DatePill(text: today),
                      const Spacer(),
                      const _LiveBadge(),
                    ],
                  ),

                  const SizedBox(height: 12),

                  MultiMetricChart(
                    series: _series,
                    visible: _visible,
                    onToggle: (k, sel) => setState(() => _visible[k] = sel),
                    legendBelow: true,
                    showNowLine: true,
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );
  }
}

class _DatePill extends StatelessWidget {
  const _DatePill({required this.text, super.key});
  final String text;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      decoration: BoxDecoration(
        color: Colors.orange,
        borderRadius: BorderRadius.circular(20),
      ),
      child: Text(
        text,
        style: const TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.w800,
          fontSize: 14,
          letterSpacing: .5,
        ),
      ),
    );
  }
}

class _LiveBadge extends StatefulWidget {
  const _LiveBadge({super.key});

  @override
  State<_LiveBadge> createState() => _LiveBadgeState();
}

class _LiveBadgeState extends State<_LiveBadge>
    with SingleTickerProviderStateMixin {
  late final AnimationController _c;
  late final Animation<double> _scale;
  late final Animation<double> _opacity;

  static const Color _liveColor = Color(0xFFDC6B02);

  @override
  void initState() {
    super.initState();
    _c = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1200),
    )..repeat();
    final curve = CurvedAnimation(parent: _c, curve: Curves.easeOut);
    _scale = Tween<double>(begin: 1.0, end: 1.8).animate(curve);
    _opacity = Tween<double>(begin: 0.6, end: 0.0).animate(curve);
  }

  @override
  void dispose() {
    _c.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        SizedBox(
          width: 18,
          height: 18,
          child: Stack(
            alignment: Alignment.center,
            children: [
              FadeTransition(
                opacity: _opacity,
                child: ScaleTransition(
                  scale: _scale,
                  child: Container(
                    decoration: const BoxDecoration(
                      color: _liveColor,
                      shape: BoxShape.circle,
                    ),
                  ),
                ),
              ),
              Container(
                width: 12,
                height: 12,
                decoration: const BoxDecoration(
                  color: _liveColor,
                  shape: BoxShape.circle,
                ),
              ),
            ],
          ),
        ),
        const SizedBox(width: 6),
        const Text(
          'LIVE',
          style: TextStyle(
            color: _liveColor,
            fontWeight: FontWeight.w800,
            letterSpacing: 1.2,
          ),
        ),
      ],
    );
  }
}
