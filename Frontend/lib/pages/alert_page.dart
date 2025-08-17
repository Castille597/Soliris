import 'package:flutter/material.dart';
import '../alert_center.dart';
import '../theme/scale_utils.dart';

class AlertPage extends StatelessWidget {
  const AlertPage({super.key});

  IconData _iconFor(String level) {
    switch (level) {
      case 'alert':
        return Icons.error_outline;
      case 'warn':
        return Icons.warning_amber_outlined;
      default:
        return Icons.info_outline;
    }
  }

  Color _colorFor(BuildContext ctx, String level) {
    switch (level) {
      case 'alert':
        return Theme.of(ctx).colorScheme.error;
      case 'warn':
        return Colors.amber.shade700;
      default:
        return Theme.of(ctx).colorScheme.primary;
    }
  }

  String _timeAgo(DateTime t) {
    final d = DateTime.now().difference(t);
    if (d.inMinutes < 1) return 'now';
    if (d.inMinutes < 60) return '${d.inMinutes} min';
    if (d.inHours < 24) return '${d.inHours} h';
    return '${d.inDays} d';
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: const Text('Alerts'),
        actions: [
          TextButton(
            onPressed: AlertCenter.instance.markAllRead,
            child: const Text(
              'Mark all read',
              style: TextStyle(color: Colors.white),
            ),
          ),
        ],
      ),
      body: ValueListenableBuilder<List<AppAlert>>(
        valueListenable: AlertCenter.instance.alertsListenable,
        builder: (context, alerts, _) {
          if (alerts.isEmpty) {
            return const Center(child: Text('No alerts yet'));
          }
          return ListView.separated(
            padding: EdgeInsets.all(16.sx(context)),
            itemCount: alerts.length,
            separatorBuilder: (_, __) => SizedBox(height: 12.sx(context)),
            itemBuilder: (context, i) {
              final a = alerts[i];
              final color = _colorFor(context, a.level);

              return Container(
                padding: EdgeInsets.all(12.sx(context)),
                decoration: BoxDecoration(
                  color: theme.cardColor,
                  borderRadius: BorderRadius.circular(12.sx(context)),
                  border: Border.all(color: color.withOpacity(.25)),
                ),
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Icon(_iconFor(a.level), color: color, size: 22.sx(context)),
                    SizedBox(width: 12.sx(context)),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            a.title,
                            style: const TextStyle(fontWeight: FontWeight.w600),
                          ),
                          SizedBox(height: 4.sx(context)),
                          Text(a.body),
                          SizedBox(height: 6.sx(context)),
                          Text(
                            _timeAgo(a.ts),
                            style: TextStyle(
                              color: Colors.grey.shade600,
                              fontSize: 12,
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              );
            },
          );
        },
      ),
    );
  }
}
