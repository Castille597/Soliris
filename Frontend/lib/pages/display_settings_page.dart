import 'package:flutter/material.dart';
import '../theme/theme_controller.dart';

class DisplaySettingsPage extends StatefulWidget {
  const DisplaySettingsPage({super.key});

  @override
  State<DisplaySettingsPage> createState() => _DisplaySettingsPageState();
}

class _DisplaySettingsPageState extends State<DisplaySettingsPage> {
  late ThemeMode _selectedMode;
  late double _previewScale;

  @override
  void initState() {
    super.initState();
    _selectedMode = ThemeController.instance.mode.value;
    _previewScale = ThemeController.instance.textScale.value;
  }

  void _changeTheme(ThemeMode? mode) {
    if (mode == null) return;
    ThemeController.instance.mode.value = mode;
    setState(() => _selectedMode = mode);
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final scheme = theme.colorScheme;

    return Scaffold(
      appBar: AppBar(
        backgroundColor: const Color(0xFFFF9800),
        elevation: 0,
        centerTitle: true,
        leading: IconButton(
          icon: const Icon(
            Icons.arrow_back_ios_new_rounded,
            color: Colors.white,
          ),
          onPressed: () => Navigator.of(context).maybePop(),
          tooltip: 'Back',
        ),
        title: const Text(
          'Display Settings',
          textScaler: TextScaler.linear(1.0),
          style: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.w800,
            fontSize: 20,
          ),
        ),
        iconTheme: const IconThemeData(color: Colors.white),
        titleTextStyle: const TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.w800,
          fontSize: 20,
        ),
      ),

      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          const Text(
            "Theme Mode",
            textScaler: TextScaler.linear(1.0),
            style: TextStyle(fontWeight: FontWeight.w700),
          ),
          const SizedBox(height: 8),

          Container(
            margin: const EdgeInsets.only(bottom: 8),
            decoration: BoxDecoration(
              color: theme.cardColor,
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: scheme.outline),
            ),
            child: RadioListTile<ThemeMode>(
              title: const Text("Light Mode"),
              value: ThemeMode.light,
              groupValue: _selectedMode,
              onChanged: _changeTheme,
              activeColor: scheme.primary,
              contentPadding: const EdgeInsets.symmetric(horizontal: 16),
            ),
          ),

          Container(
            margin: const EdgeInsets.only(bottom: 24),
            decoration: BoxDecoration(
              color: theme.cardColor,
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: scheme.outline),
            ),
            child: RadioListTile<ThemeMode>(
              title: const Text("Dark Mode"),
              value: ThemeMode.dark,
              groupValue: _selectedMode,
              onChanged: _changeTheme,
              activeColor: scheme.primary,
              contentPadding: const EdgeInsets.symmetric(horizontal: 16),
            ),
          ),

          const Text(
            "Text Size",
            textScaler: TextScaler.linear(1.0),
            style: TextStyle(fontWeight: FontWeight.w700),
          ),
          Slider(
            value: _previewScale,
            min: 1.0,
            max: 1.8,
            divisions: 8,
            label: "${(_previewScale * 100).round()}%",
            onChanged: (v) => setState(() => _previewScale = v),
            onChangeEnd: (v) => ThemeController.instance.setTextScale(v),
            activeColor: scheme.primary,
            inactiveColor: scheme.outline.withOpacity(.6),
          ),
          const SizedBox(height: 8),

          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: theme.cardColor,
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: scheme.outline),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  "Preview text at ${(_previewScale * 100).round()}%",
                  style: theme.textTheme.bodyMedium?.copyWith(
                    fontSize: 16 * _previewScale,
                    fontWeight: FontWeight.w600,
                  ),
                ),
                const SizedBox(height: 12),
                Text(
                  "This is how regular body text will look.",
                  style: theme.textTheme.bodyMedium?.copyWith(
                    fontSize: 14 * _previewScale,
                  ),
                ),
                const SizedBox(height: 8),
                Text(
                  "And this could be used for titles.",
                  style: theme.textTheme.titleLarge?.copyWith(
                    fontSize: 20 * _previewScale,
                    fontWeight: FontWeight.w800,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
