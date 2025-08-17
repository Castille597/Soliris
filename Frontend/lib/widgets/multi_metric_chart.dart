import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';

class MultiMetricChart extends StatelessWidget {
  const MultiMetricChart({
    super.key,
    required this.series,
    required this.visible,
    required this.onToggle,
    this.height = 300,
    this.padding = const EdgeInsets.all(12),
    this.fillUnderLine = true,
    this.showNowLine = false,
    this.yMin = 0,
    this.yMax = 250,
    this.yStep = 50,
    this.legendBelow = false,
  });

  final Map<String, List<FlSpot>> series;
  final Map<String, bool> visible;
  final void Function(String key, bool selected) onToggle;
  final double height;
  final EdgeInsetsGeometry padding;
  final bool fillUnderLine;
  final bool showNowLine;

  final double yMin;
  final double yMax;
  final double yStep;

  final bool legendBelow;

  static double minutesSinceMidnight(DateTime t) =>
      (t.hour * 60 + t.minute + t.second / 60.0);

  static DateTime parseTimestamp(dynamic ts) {
    if (ts is int) {
      final ms = ts > 20000000000 ? ts : ts * 1000;
      return DateTime.fromMillisecondsSinceEpoch(ms, isUtc: true).toLocal();
    }
    if (ts is String) return DateTime.parse(ts).toLocal();
    return DateTime.now();
  }

  String _canon(String key) {
    final k = key.trim().toLowerCase();
    if (k == 'hr' || k == 'heart rate') return 'HR';
    if (k == 'spo2' || k == 'spo₂' || k == 'blood oxygen') return 'SpO₂';
    if (k == 'skin temp' || k == 'skin temperature' || k == 'temp_skin') {
      return 'Skin temp';
    }
    if (k == 'resp rate' || k == 'respiratory rate') return 'Resp rate';
    if (k == 'steps' || k == 'steps/min' || k == 'step_rate')
      return 'Steps/min';
    return key;
  }

  Map<String, List<FlSpot>> _mergedSeries() {
    final Map<String, List<FlSpot>> out = <String, List<FlSpot>>{};
    series.forEach((rawKey, pts) {
      final c = _canon(rawKey);
      final list = out.putIfAbsent(c, () => <FlSpot>[]);
      list.addAll(pts);
    });
    for (final e in out.entries) {
      e.value.sort((a, b) => a.x.compareTo(b.x));
      final dedup = <FlSpot>[];
      double? lastX;
      for (final p in e.value) {
        if (lastX == null || p.x != lastX) {
          dedup.add(p);
          lastX = p.x;
        } else {
          dedup[dedup.length - 1] = p;
        }
      }
      out[e.key] = dedup;
    }
    return out;
  }

  Color _colorForKey(BuildContext ctx, String key) {
    switch (_canon(key)) {
      case 'HR':
        return Colors.orange;
      case 'SpO₂':
        return Colors.teal;
      case 'Skin temp':
        return Colors.brown;
      case 'Resp rate':
        return const Color(0xFFFF6B9A);
      case 'Steps/min':
        return const Color(0xFF9B88FF);
      default:
        return Theme.of(ctx).colorScheme.primary;
    }
  }

  List<LineChartBarData> _lines(
    BuildContext context,
    Map<String, List<FlSpot>> merged,
  ) {
    final out = <LineChartBarData>[];
    for (final e in merged.entries) {
      if (visible[e.key] != true) continue;
      final pts = e.value;
      if (pts.isEmpty) continue;

      final c = _colorForKey(context, e.key);
      out.add(
        LineChartBarData(
          spots: pts,
          isCurved: true,
          barWidth: 2.5,
          isStrokeCapRound: true,
          color: c,
          dotData: const FlDotData(show: false),
          belowBarData: fillUnderLine
              ? BarAreaData(
                  show: true,
                  gradient: LinearGradient(
                    colors: [c.withOpacity(0.18), c.withOpacity(0.0)],
                    begin: Alignment.topCenter,
                    end: Alignment.bottomCenter,
                  ),
                )
              : BarAreaData(show: false),
        ),
      );
    }
    return out;
  }

  Widget _legend(BuildContext context, Map<String, List<FlSpot>> merged) {
    final keys = merged.keys.toList();
    const softBorder = Color(0xFFFFE0B2);
    final Color baseText = Theme.of(context).colorScheme.onSurface;

    return Wrap(
      spacing: 8,
      runSpacing: 8,
      children: [
        for (final k in keys)
          FilterChip(
            label: Text(
              k,
              style: TextStyle(color: baseText, fontWeight: FontWeight.w700),
            ),
            selected: visible[k] ?? true,
            onSelected: (sel) => onToggle(k, sel),
            avatar: CircleAvatar(
              radius: 6,
              backgroundColor: _colorForKey(context, k),
            ),
            showCheckmark: false,
            selectedColor: Colors.orange.withOpacity(.15),
            backgroundColor: Theme.of(context).cardColor,
            checkmarkColor: Colors.orange,
            side: const BorderSide(color: softBorder),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
            ),
          ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    final merged = _mergedSeries();
    final bars = _lines(context, merged);

    const double minX = 0.0;
    const double maxX = 1440.0;
    final nowMin = minutesSinceMidnight(DateTime.now());
    const hourTicks = <int>[0, 4, 8, 12, 16, 20, 24];

    final legend = _legend(context, merged);

    final chart = Container(
      height: height,
      padding: padding,
      decoration: BoxDecoration(
        color: theme.cardColor,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.orange.shade200, width: 1),
      ),
      child: bars.isEmpty
          ? Center(
              child: Text(
                'No data (select a metric)',
                style: theme.textTheme.bodyMedium,
              ),
            )
          : LineChart(
              LineChartData(
                minX: minX,
                maxX: maxX,
                minY: yMin,
                maxY: yMax,
                gridData: FlGridData(
                  show: true,
                  drawVerticalLine: false,
                  horizontalInterval: yStep,
                  getDrawingHorizontalLine: (v) => FlLine(
                    color: Colors.grey.withOpacity(0.18),
                    strokeWidth: 1,
                  ),
                ),
                titlesData: FlTitlesData(
                  topTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false),
                  ),
                  rightTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false),
                  ),
                  bottomTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 22,
                      interval: 60,
                      getTitlesWidget: (value, meta) {
                        final m = value.round().clamp(0, 1440);
                        final h = (m / 60).round();
                        if (!hourTicks.contains(h))
                          return const SizedBox.shrink();
                        return const Text(
                          'h',
                          style: TextStyle(fontSize: 0),
                        ).build(context);
                      },
                    ),
                  ),
                  leftTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 36,
                      interval: yStep,
                      getTitlesWidget: (value, meta) => Text(
                        value.toStringAsFixed(0),
                        style: const TextStyle(fontSize: 10),
                      ),
                    ),
                  ),
                ),
                borderData: FlBorderData(
                  show: true,
                  border: Border.all(color: Colors.grey.shade400, width: 1),
                ),
                extraLinesData: showNowLine
                    ? ExtraLinesData(
                        verticalLines: [
                          VerticalLine(
                            x: nowMin,
                            color: Colors.orange.withOpacity(0.6),
                            strokeWidth: 1.5,
                            dashArray: const [6, 4],
                          ),
                        ],
                      )
                    : const ExtraLinesData(),
                lineBarsData: bars,
              ),
            ),
    );

    final children = legendBelow
        ? <Widget>[chart, const SizedBox(height: 12), legend]
        : <Widget>[legend, const SizedBox(height: 12), chart];

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: children,
    );
  }
}
