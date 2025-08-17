import 'dart:async';
import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';
import 'multi_metric_chart.dart';

class HealthMetricsSection extends StatefulWidget {
  const HealthMetricsSection({
    super.key,
    this.telemetryStream,
    this.simulateIfNoStream = true,
    this.height = 320,
  });

  final Stream<Map<String, dynamic>>? telemetryStream;
  final bool simulateIfNoStream;
  final double height;

  @override
  State<HealthMetricsSection> createState() => _HealthMetricsSectionState();
}

class _HealthMetricsSectionState extends State<HealthMetricsSection> {
  final Map<String, List<FlSpot>> series = {
    'HR': <FlSpot>[],
    'SpO₂': <FlSpot>[],
    'Skin temp': <FlSpot>[],
  };

  final Map<String, bool> visible = {
    'HR': true,
    'SpO₂': true,
    'Skin temp': true,
  };

  StreamSubscription<Map<String, dynamic>>? _sub;
  Timer? _sim;
  DateTime _currentDay = DateTime.now();

  @override
  void initState() {
    super.initState();

    if (widget.telemetryStream != null) {
      _sub = widget.telemetryStream!.listen(ingestTelemetry);
    } else if (widget.simulateIfNoStream) {
      _sim = Timer.periodic(const Duration(seconds: 10), (_) {
        final now = DateTime.now();
        ingestTelemetry({
          'timestamp': now.toIso8601String(),
          'hr': 70 + (now.second % 12),
          'spo2': 97,
          'skin_temp': 33.5 + ((now.second % 6) * 0.05),
        });
      });
    }
  }

  @override
  void dispose() {
    _sub?.cancel();
    _sim?.cancel();
    super.dispose();
  }

  void ingestTelemetry(Map<String, dynamic> j) {
    _maybeResetAtMidnight();

    final tsRaw = j['timestamp'] ?? j['ts'] ?? j['time'];
    final ts = MultiMetricChart.parseTimestamp(tsRaw);
    final x = MultiMetricChart.minutesSinceMidnight(ts);

    final num? hr = j['hr'] ?? j['heart_rate'];
    final num? spo2 = j['spo2'] ?? j['SpO2'];
    final num? tskin = j['skin_temp'] ?? j['temp_skin'];

    if (hr != null) _addPoint('HR', x, hr.toDouble());
    if (spo2 != null) _addPoint('SpO₂', x, spo2.toDouble());
    if (tskin != null) _addPoint('Skin temp', x, tskin.toDouble());

    if (mounted) setState(() {});
  }

  void _addPoint(String key, double x, double y) {
    final list = series[key] ??= <FlSpot>[];
    list.add(FlSpot(x, y));
    list.removeWhere((p) => p.x < 0 || p.x > 1440);
    list.sort((a, b) => a.x.compareTo(b.x));
    if (list.length > 2000) list.removeRange(0, list.length - 2000);
  }

  void _maybeResetAtMidnight() {
    final now = DateTime.now();
    if (now.day != _currentDay.day ||
        now.month != _currentDay.month ||
        now.year != _currentDay.year) {
      for (final k in series.keys) {
        series[k] = <FlSpot>[];
      }
      _currentDay = now;
    }
  }

  @override
  Widget build(BuildContext context) {
    return MultiMetricChart(
      series: series,
      visible: visible,
      onToggle: (k, sel) => setState(() => visible[k] = sel),
      height: widget.height,
      showNowLine: true,
    );
  }
}
