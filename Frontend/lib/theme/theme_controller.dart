import 'package:flutter/material.dart';

class ThemeController {
  ThemeController._();
  static final instance = ThemeController._();

  final ValueNotifier<ThemeMode> mode = ValueNotifier(ThemeMode.system);

  final ValueNotifier<double> textScale = ValueNotifier<double>(1.0);
  void setTextScale(double v) => textScale.value = v.clamp(0.9, 1.6);

  ThemeData get light => _buildTheme(Brightness.light);
  ThemeData get dark => _buildTheme(Brightness.dark);

  void toggle() {
    mode.value = mode.value == ThemeMode.dark
        ? ThemeMode.light
        : ThemeMode.dark;
  }

  static const _brand = Color(0xFFFF9800);
  static const _brandDark = Color(0xFFFFB74D);

  ThemeData _buildTheme(Brightness b) {
    final isDark = b == Brightness.dark;

    final bg = isDark ? const Color(0xFF121212) : const Color(0xFFFFFFFF);
    final surf = isDark ? const Color(0xFF1C1C1C) : const Color(0xFFFFF4E8);
    final card = isDark ? const Color(0xFF222222) : const Color(0xFFFFF0DE);
    final border = isDark ? const Color(0xFF2F2F2F) : const Color(0xFFFFE0B2);

    final scheme = ColorScheme(
      brightness: b,
      primary: isDark ? _brandDark : _brand,
      onPrimary: Colors.white,
      secondary: isDark ? const Color(0xFFFFCC80) : const Color(0xFFFFB84D),
      onSecondary: Colors.black,
      surface: surf,
      onSurface: isDark ? Colors.white : const Color(0xFF111111),
      surfaceContainerHighest: card,
      surfaceContainerHigh: card,
      surfaceContainer: card,
      surfaceContainerLow: card,
      surfaceContainerLowest: card,
      background: bg,
      onBackground: isDark ? Colors.white : const Color(0xFF111111),
      error: const Color(0xFFB00020),
      onError: Colors.white,
      tertiary: isDark ? const Color(0xFF9B88FF) : const Color(0xFF6E56CF),
      onTertiary: Colors.white,
      outline: border,
      outlineVariant: border.withOpacity(.8),
      shadow: Colors.black,
      scrim: Colors.black54,
    );

    return ThemeData(
      useMaterial3: true,
      colorScheme: scheme,
      brightness: b,
      scaffoldBackgroundColor: bg,
      canvasColor: bg,
      cardColor: card,
      dividerColor: border,
      appBarTheme: AppBarTheme(
        elevation: 0,
        backgroundColor: bg,
        centerTitle: true,
        iconTheme: IconThemeData(color: scheme.primary),
        titleTextStyle: TextStyle(
          color: scheme.primary,
          fontWeight: FontWeight.w800,
          fontSize: 22,
        ),
      ),
      chipTheme: ChipThemeData(
        backgroundColor: isDark
            ? const Color(0xFF262626)
            : const Color(0xFFF3E6D6),
        selectedColor: scheme.primary.withOpacity(.18),
        labelStyle: TextStyle(
          color: scheme.onSurface,
          fontWeight: FontWeight.w700,
        ),
        side: BorderSide(color: border),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      ),
      textTheme: (isDark ? ThemeData.dark() : ThemeData.light()).textTheme
          .apply(bodyColor: scheme.onSurface, displayColor: scheme.onSurface),
      iconTheme: IconThemeData(color: scheme.primary),
      bottomNavigationBarTheme: BottomNavigationBarThemeData(
        backgroundColor: Colors.transparent,
        selectedItemColor: scheme.primary,
        unselectedItemColor: scheme.onSurface.withOpacity(.6),
        showUnselectedLabels: true,
        type: BottomNavigationBarType.fixed,
      ),
    );
  }
}
