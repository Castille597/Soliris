import 'package:flutter/material.dart';
import '../alert_center.dart';
import 'notification_bell.dart';
import '../pages/alert_page.dart';

class AppTopBar extends StatelessWidget {
  const AppTopBar({super.key});

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
      child: Row(
        children: [
          ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: Image.asset(
              'assets/images/unnamed.jpg',
              height: 60,
              width: 60,
              fit: BoxFit.cover,
              errorBuilder: (_, __, ___) => Icon(
                Icons.image_not_supported,
                size: 28,
                color: cs.onSurface.withOpacity(.6),
              ),
            ),
          ),

          Expanded(
            child: Center(
              child: Text(
                'Soliris',
                textScaleFactor: 1.0,
                overflow: TextOverflow.fade,
                softWrap: false,
                style: TextStyle(
                  fontSize: 30,
                  fontWeight: FontWeight.bold,
                  color: cs.primary,
                ),
              ),
            ),
          ),

          NotificationBell(
            color: cs.primary,
            iconSize: 35,
            onPressed: () async {
              await Navigator.of(
                context,
              ).push(MaterialPageRoute(builder: (_) => const AlertPage()));
              AlertCenter.instance.markAllRead();
            },
          ),
        ],
      ),
    );
  }
}
