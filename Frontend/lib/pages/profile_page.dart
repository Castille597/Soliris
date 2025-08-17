import 'package:flutter/material.dart';
import 'package:characters/characters.dart';

import 'led_vibrations_page.dart';
import 'wifi_settings_page.dart';
import 'privacy_settings_page.dart';
import 'edit_profile_page.dart';
import 'display_settings_page.dart';

import '../widgets/app_top_bar.dart';
import '../profile_store.dart';

class ProfilePage extends StatelessWidget {
  const ProfilePage({super.key});

  String _initialOf(String s) {
    final t = s.trim();
    if (t.isEmpty) return '?';
    return t.characters.first.toUpperCase();
  }

  static String _genderLabel(String key) {
    switch (key.toLowerCase()) {
      case 'woman':
        return 'Woman';
      case 'man':
        return 'Man';
      default:
        return 'Other';
    }
  }

  void _showSavedBanner(BuildContext context) {
    final theme = Theme.of(context);
    ScaffoldMessenger.of(context)
      ..hideCurrentMaterialBanner()
      ..showMaterialBanner(
        MaterialBanner(
          backgroundColor: theme.cardColor,
          elevation: 0,
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
          leading: const Icon(Icons.check_circle, color: Colors.green),
          content: Text(
            'Information successfully saved!',
            style: theme.textTheme.bodyMedium?.copyWith(
              fontWeight: FontWeight.w700,
            ),
          ),
          actions: [
            TextButton(
              onPressed: () =>
                  ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
              child: const Text('DISMISS'),
            ),
          ],
        ),
      );
    Future.delayed(const Duration(seconds: 3), () {
      ScaffoldMessenger.of(context).hideCurrentMaterialBanner();
    });
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,

      appBar: const PreferredSize(
        preferredSize: Size.fromHeight(70),
        child: AppTopBar(),
      ),

      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.fromLTRB(20, 10, 20, 24),
          children: [
            ValueListenableBuilder<UserProfile?>(
              valueListenable: ProfileStore.instance.profile,
              builder: (context, p, _) {
                final displayName = (p?.displayName ?? '').trim();
                final subtitle = [
                  if ((p?.gender ?? '').isNotEmpty) _genderLabel(p!.gender),
                  if (p?.age != null) '${p!.age} y.',
                ].join(' • ');
                final avatarLetter = _initialOf(
                  displayName.isNotEmpty ? displayName : ' ',
                );

                return Container(
                  margin: const EdgeInsets.only(bottom: 20),
                  padding: const EdgeInsets.all(20),
                  decoration: BoxDecoration(
                    color: theme.cardColor,
                    borderRadius: BorderRadius.circular(30),
                  ),
                  child: Row(
                    children: [
                      CircleAvatar(
                        radius: 30,
                        backgroundColor: Colors.orange,
                        child: Text(
                          avatarLetter,
                          style: const TextStyle(
                            fontSize: 24,
                            color: Colors.white,
                          ),
                        ),
                      ),
                      const SizedBox(width: 20),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              displayName.isEmpty ? 'Profile' : displayName,
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                              style: TextStyle(
                                fontSize: 18,
                                fontWeight: FontWeight.bold,
                                color: theme.textTheme.bodyMedium?.color,
                              ),
                            ),
                            const SizedBox(height: 2),
                            Text(
                              subtitle.isEmpty ? 'Profile Settings' : subtitle,
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                              style: TextStyle(
                                color: theme.textTheme.bodyMedium?.color
                                    ?.withOpacity(0.7),
                              ),
                            ),
                          ],
                        ),
                      ),
                      IconButton(
                        icon: const Icon(Icons.edit, color: Colors.orange),
                        onPressed: () async {
                          final saved = await Navigator.push(
                            context,
                            MaterialPageRoute(
                              builder: (_) => const EditProfilePage(),
                            ),
                          );
                          if (saved == true) _showSavedBanner(context);
                        },
                      ),
                    ],
                  ),
                );
              },
            ),

            Container(
              margin: const EdgeInsets.only(bottom: 20),
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: theme.cardColor,
                borderRadius: BorderRadius.circular(30),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Align(
                    alignment: Alignment.centerRight,
                    child: Text(
                      'Connected',
                      style: const TextStyle(
                        color: Colors.orangeAccent,
                        fontSize: 20,
                        fontWeight: FontWeight.w700,
                        letterSpacing: .2,
                      ),
                    ),
                  ),
                  const SizedBox(height: 8),
                  Row(
                    children: [
                      const Icon(Icons.sync, size: 50, color: Colors.orange),
                      const SizedBox(width: 12),
                      Text(
                        'Paired Bracelet',
                        style: TextStyle(
                          fontSize: 14,
                          color: theme.textTheme.bodyMedium?.color,
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),

            const SettingsTile(
              icon: Icons.flash_on,
              title: 'LED/VIBRATIONS',
              page: LedVibrationsPage(),
            ),
            const SettingsTile(
              icon: Icons.wifi,
              title: 'WIFI',
              page: WifiSettingsPage(),
            ),
            const SettingsTile(
              icon: Icons.brightness_5,
              title: 'DISPLAY',
              page: DisplaySettingsPage(),
            ),
            const SettingsTile(
              icon: Icons.lock_outline,
              title: 'PRIVACY',
              page: PrivacySettingsPage(),
            ),
          ],
        ),
      ),
    );
  }
}

class SettingsTile extends StatelessWidget {
  final IconData icon;
  final String title;
  final Widget page;

  const SettingsTile({
    super.key,
    required this.icon,
    required this.title,
    required this.page,
  });

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return GestureDetector(
      onTap: () =>
          Navigator.push(context, MaterialPageRoute(builder: (_) => page)),
      child: Container(
        margin: const EdgeInsets.only(bottom: 20),
        padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 18),
        decoration: BoxDecoration(
          color: theme.cardColor,
          borderRadius: BorderRadius.circular(30),
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Row(
              children: [
                Icon(icon, color: Colors.orange, size: 28),
                const SizedBox(width: 16),
                Text(
                  title,
                  style: TextStyle(
                    fontSize: 16,
                    letterSpacing: 1.1,
                    fontWeight: FontWeight.w500,
                    color: theme.textTheme.bodyMedium?.color,
                  ),
                ),
              ],
            ),
            const Icon(Icons.play_arrow, color: Colors.orange, size: 28),
          ],
        ),
      ),
    );
  }
}
