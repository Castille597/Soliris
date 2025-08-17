import 'dart:async';
import 'package:flutter/material.dart';

class DaySelector extends StatefulWidget {
  const DaySelector({
    super.key,
    this.startOnMonday = false,
    this.onSelected,
    this.initialSelectedDate,
  });

  final bool startOnMonday;
  final ValueChanged<DateTime>? onSelected;
  final DateTime? initialSelectedDate;

  @override
  State<DaySelector> createState() => _DaySelectorState();
}

class _DaySelectorState extends State<DaySelector> {
  late DateTime _selected;
  Timer? _midnightTimer;

  @override
  void initState() {
    super.initState();
    _selected = _stripTime(widget.initialSelectedDate ?? DateTime.now());
    _scheduleMidnightTick();
  }

  @override
  void dispose() {
    _midnightTimer?.cancel();
    super.dispose();
  }

  static DateTime _stripTime(DateTime d) => DateTime(d.year, d.month, d.day);

  void _scheduleMidnightTick() {
    _midnightTimer?.cancel();
    final now = DateTime.now();
    final nextMidnight = DateTime(now.year, now.month, now.day + 1);
    final delay = nextMidnight.difference(now) + const Duration(seconds: 1);
    _midnightTimer = Timer(delay, () {
      if (!mounted) return;
      setState(() {});
      _scheduleMidnightTick();
    });
  }

  List<DateTime> _currentWeekDays() {
    final now = _stripTime(DateTime.now());
    final wd = now.weekday;
    final delta = widget.startOnMonday ? (wd - 1) : (wd % 7);
    final start = now.subtract(Duration(days: delta));
    return List.generate(7, (i) => start.add(Duration(days: i)));
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;

    final days = _currentWeekDays();
    final dayLabels = widget.startOnMonday
        ? const ["Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"]
        : const ["Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"];

    final double globalScale = MediaQuery.of(context).textScaleFactor;
    final double numberScale = globalScale.clamp(0.0, 1.5);

    return Container(
      padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 12),
      decoration: BoxDecoration(
        color: theme.cardColor,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: cs.outline),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: List.generate(7, (i) {
          final date = days[i];
          final isToday = _stripTime(DateTime.now()) == date;
          final isSelected = _selected == date;

          final String label = dayLabels[i];
          final String dayNum = date.day.toString();

          final Color dowColor = isSelected ? cs.primary : cs.onSurface;
          final Color circleBg = isSelected ? cs.primary : Colors.transparent;
          final Color circleText = isSelected ? cs.onPrimary : cs.onSurface;
          final BoxBorder? circleBorder = isSelected
              ? null
              : Border.all(color: cs.outline);

          return GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: () {
              setState(() => _selected = date);
              widget.onSelected?.call(date);
            },
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(
                  label,
                  style: theme.textTheme.titleMedium?.copyWith(
                    fontWeight: FontWeight.w700,
                    color: dowColor,
                  ),
                ),
                const SizedBox(height: 6),

                Container(
                  width: 36,
                  height: 36,
                  alignment: Alignment.center,
                  decoration: BoxDecoration(
                    color: circleBg,
                    shape: BoxShape.circle,
                    border: circleBorder,
                  ),
                  child: MediaQuery(
                    data: MediaQuery.of(
                      context,
                    ).copyWith(textScaler: TextScaler.linear(numberScale)),
                    child: Text(
                      dayNum,
                      style: theme.textTheme.titleMedium?.copyWith(
                        color: circleText,
                        fontWeight: FontWeight.w800,
                      ),
                    ),
                  ),
                ),

                if (isToday && !isSelected) ...[
                  const SizedBox(height: 4),
                  Container(
                    width: 6,
                    height: 6,
                    decoration: BoxDecoration(
                      color: cs.primary,
                      shape: BoxShape.circle,
                    ),
                  ),
                ],
              ],
            ),
          );
        }),
      ),
    );
  }
}
