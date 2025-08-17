import 'package:flutter/material.dart';
import 'package:characters/characters.dart';
import '../profile_store.dart';

class GreetingHeader extends StatelessWidget {
  const GreetingHeader({super.key});

  String _greeting() {
    final h = DateTime.now().hour;
    if (h < 6) return 'Good Night';
    if (h < 12) return 'Good Morning';
    if (h < 18) return 'Good Afternoon';
    return 'Good Evening';
  }

  String _titleCase(String s) {
    final t = s.trim();
    if (t.isEmpty) return t;
    final chars = t.characters;
    final first = chars.first.toUpperCase();
    final rest = chars.skip(1).toString();
    return '$first$rest';
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return ValueListenableBuilder<UserProfile?>(
      valueListenable: ProfileStore.instance.profile,
      builder: (context, profile, _) {
        final name = _titleCase(profile?.displayName ?? '');
        final line = name.isNotEmpty ? '${_greeting()} $name ✨' : _greeting();

        return Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              line,
              style: theme.textTheme.titleLarge?.copyWith(
                fontSize: 22,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 4),
            Text(
              'How are you feeling today?',
              style: theme.textTheme.bodyMedium,
            ),
          ],
        );
      },
    );
  }
}
