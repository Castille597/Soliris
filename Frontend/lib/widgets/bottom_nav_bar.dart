import 'package:flutter/material.dart';

class BottomNavBar extends StatelessWidget {
  final int currentIndex;
  final Function(int) onTap;

  const BottomNavBar({
    super.key,
    required this.currentIndex,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    final List<IconData> icons = const [
      Icons.home_rounded,
      Icons.insert_drive_file_rounded,
      Icons.wb_sunny_rounded,
      Icons.person_rounded,
    ];

    final List<Color> gradientColors = isDark
        ? const [Color(0xFF2A2A2A), Color(0xFF1C1C1C)]
        : const [Color(0xFFFFF1CC), Color(0xFFFFC27A)];

    final Color selectedColor = theme.colorScheme.primary;
    final Color unselectedColor = isDark ? Colors.white70 : Colors.white;

    return Container(
      margin: const EdgeInsets.only(top: 16),
      padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 10),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: gradientColors,
          begin: Alignment.centerLeft,
          end: Alignment.centerRight,
        ),
        borderRadius: BorderRadius.circular(24),
      ),
      child: SafeArea(
        top: false,
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: List.generate(icons.length, (index) {
            final bool isSelected = index == currentIndex;

            return Material(
              color: Colors.transparent,
              child: InkWell(
                borderRadius: BorderRadius.circular(50),
                onTap: () {
                  debugPrint('BottomNav tapped $index');
                  onTap(index);
                },
                child: Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Icon(
                    icons[index],
                    color: isSelected ? selectedColor : unselectedColor,
                    size: isSelected ? 32 : 26,
                  ),
                ),
              ),
            );
          }),
        ),
      ),
    );
  }
}
