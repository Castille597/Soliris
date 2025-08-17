import 'package:flutter/material.dart';

class HealthMetricCard extends StatelessWidget {
  const HealthMetricCard({
    super.key,
    required this.title,
    required this.value,
    required this.icon,
    this.valueColor,
  });

  final String title;
  final String value;
  final IconData icon;
  final Color? valueColor;

  double _scale(BuildContext c) =>
      MediaQuery.of(c).textScaleFactor.clamp(0.9, 1.6);

  double _minWidth(BuildContext c) => 170.0 * _scale(c);

  double _minHeight(BuildContext c) {
    final s = _scale(c);
    return 120.0 + (s - 1.0) * 80.0;
  }

  EdgeInsets _pad(BuildContext c) {
    final s = _scale(c);
    return EdgeInsets.all(14.0 + (s - 1.0) * 6.0);
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;
    final s = _scale(context);

    return ConstrainedBox(
      constraints: BoxConstraints(
        minWidth: _minWidth(context),
        minHeight: _minHeight(context),
      ),
      child: Container(
        padding: _pad(context),
        decoration: BoxDecoration(
          color: theme.cardColor,
          borderRadius: BorderRadius.circular(20),
          border: Border.all(color: cs.outline),
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Icon(icon, color: cs.primary, size: 24 * s),
            const SizedBox(height: 8),
            Text(
              title,
              maxLines: 2,
              overflow: TextOverflow.fade,
              style: theme.textTheme.titleMedium?.copyWith(
                color: cs.primary,
                fontWeight: FontWeight.w700,
              ),
            ),
            const SizedBox(height: 10),
            Align(
              alignment: Alignment.bottomLeft,
              child: Text(
                value,
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: theme.textTheme.titleLarge?.copyWith(
                  fontWeight: FontWeight.w900,
                  letterSpacing: .2,
                  color: valueColor ?? theme.textTheme.titleLarge?.color,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
