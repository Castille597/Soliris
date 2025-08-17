import 'package:flutter/material.dart';
import '../alert_center.dart';

class NotificationBell extends StatelessWidget {
  const NotificationBell({
    super.key,
    this.onPressed,
    this.color,
    this.iconSize = 26,
    this.cap = 9,
  });

  final VoidCallback? onPressed;
  final Color? color;
  final double iconSize;
  final int cap;

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<int>(
      valueListenable: AlertCenter.instance.unread,
      builder: (context, count, _) {
        final theme = Theme.of(context);
        final cs = theme.colorScheme;
        final isDark = theme.brightness == Brightness.dark;

        final Color bellColor = color ?? cs.primary;

        final Color badgeBg = isDark ? Colors.white : cs.primary;
        final Color badgeFg = isDark ? cs.primary : Colors.white;

        const double kBadgeSide = 20.0;
        const double kBadgeFont = 11.0;
        const double kOffset = 6.0;
        const double kBorderW = 1.0;

        final String text = count > cap ? '$cap+' : '$count';

        return Stack(
          clipBehavior: Clip.none,
          children: [
            IconButton(
              iconSize: iconSize,
              icon: Icon(Icons.notifications_none, color: bellColor),
              onPressed: onPressed,
              tooltip: 'Notifications',
            ),
            if (count > 0)
              Positioned(
                right: kOffset,
                top: kOffset,
                child: Semantics(
                  label: '$count unread notifications',
                  container: true,
                  child: Container(
                    width: kBadgeSide,
                    height: kBadgeSide,
                    alignment: Alignment.center,
                    decoration: BoxDecoration(
                      color: badgeBg,
                      shape: BoxShape.circle,
                      border: isDark
                          ? Border.all(color: cs.primary, width: kBorderW)
                          : null,
                    ),
                    child: Text(
                      text,
                      textScaleFactor: 1.0,
                      style: TextStyle(
                        color: badgeFg,
                        fontSize: kBadgeFont,
                        fontWeight: FontWeight.w700,
                        height: 1.0,
                      ),
                    ),
                  ),
                ),
              ),
          ],
        );
      },
    );
  }
}
