import 'package:flutter/material.dart';
import 'dart:async';
import '../realtime_service.dart' show realtime;
import 'health_metric_card.dart';

class LiveActivityRow extends StatefulWidget {
  const LiveActivityRow({super.key});

  @override
  State<LiveActivityRow> createState() => _LiveActivityRowState();
}

class _LiveActivityRowState extends State<LiveActivityRow> {
  StreamSubscription? _sub;
  Map<String, dynamic> _data = const {};

  @override
  void initState() {
    super.initState();
    realtime.connect();
    _sub = realtime.stream.listen((m) {
      setState(() => _data = m);
    });
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  int? _parseSteps(Map<String, dynamic> m) {
    final v = m['steps'] ?? m['step_count'];
    if (v == null) return null;
    if (v is num) return v.toInt();
    return int.tryParse('$v');
  }

  String? _parsePosture(Map<String, dynamic> m) {
    final v = (m['posture'] ?? '').toString().toLowerCase();
    switch (v) {
      case 'standing':
        return 'Standing';
      case 'sitting':
        return 'Sitting';
      case 'lying':
        return 'Lying';
    }
    return null;
  }

  String? _parseActivity(Map<String, dynamic> m) {
    final v = (m['activity'] ?? '').toString().toLowerCase();
    switch (v) {
      case 'walk':
        return 'Walk';
      case 'run':
        return 'Run';
      case 'still':
        return 'Still';
    }
    return null;
  }

  @override
  Widget build(BuildContext context) {
    final steps = _parseSteps(_data);
    final posture = _parsePosture(_data) ?? '—';
    final activity = _parseActivity(_data) ?? '—';

    Widget metricCard({
      required String title,
      required String value,
      required IconData icon,
    }) {
      return Expanded(
        child: Card(
          elevation: 0,
          color: Theme.of(context).colorScheme.surface,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(icon, size: 20),
                const SizedBox(height: 6),
                Text(title, style: Theme.of(context).textTheme.labelMedium),
                const SizedBox(height: 4),
                Text(
                  value,
                  style: Theme.of(context).textTheme.titleMedium?.copyWith(
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ],
            ),
          ),
        ),
      );
    }

    return Row(
      children: [
        metricCard(
          title: 'Steps',
          value: steps?.toString() ?? '--',
          icon: Icons.directions_walk,
        ),
        const SizedBox(width: 12),
        metricCard(
          title: 'Posture',
          value: posture,
          icon: Icons.accessibility_new,
        ),
        const SizedBox(width: 12),
        metricCard(
          title: 'Activity',
          value: activity,
          icon: Icons.directions_run,
        ),
      ],
    );
  }
}
