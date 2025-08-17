import 'package:flutter/material.dart';
import '../realtime_service.dart';

class MoodSelector extends StatefulWidget {
  const MoodSelector({super.key});

  @override
  State<MoodSelector> createState() => _MoodSelectorState();
}

class _MoodSelectorState extends State<MoodSelector> {
  int selectedMood = -1;

  final List<IconData> moodIcons = const [
    Icons.sentiment_very_satisfied,
    Icons.sentiment_satisfied,
    Icons.sentiment_neutral,
    Icons.sentiment_dissatisfied,
  ];

  final List<Color> moodColors = const [
    Colors.green,
    Color.fromARGB(255, 250, 233, 0),
    Colors.orange,
    Colors.red,
  ];

  final List<String> moodLabels = const ['Very good', 'Good', 'Average', 'Bad'];

  final List<String> moodKeys = const ['very_good', 'good', 'average', 'bad'];

  Future<void> _onSelectMood(int index) async {
    setState(() => selectedMood = index);
    try {
      await sendMood(moodKeys[index], userId: realtime.userId);
    } catch (_) {}
  }

  @override
  Widget build(BuildContext context) {
    final mq = MediaQuery.of(context);
    return MediaQuery(
      data: mq.copyWith(textScaler: const TextScaler.linear(1.0)),
      child: LayoutBuilder(
        builder: (context, constraints) {
          const double iconSize = 40;
          const double pad = 12;
          const double borderW = 2;
          const double labelFont = 16;
          const double gap = 8;
          const double hSpacing = 16;

          final double colWidth = (constraints.maxWidth - 3 * hSpacing) / 4;

          Widget item(int index) {
            final bool isSelected = index == selectedMood;
            final Color color = isSelected ? moodColors[index] : Colors.grey;

            return SizedBox(
              width: colWidth,
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  GestureDetector(
                    onTap: () => _onSelectMood(index),
                    child: Container(
                      padding: const EdgeInsets.all(pad),
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: isSelected
                            ? color.withOpacity(0.20)
                            : Colors.grey.shade100,
                        border: Border.all(
                          color: isSelected ? color : Colors.grey.shade300,
                          width: borderW,
                        ),
                      ),
                      child: Icon(
                        moodIcons[index],
                        size: iconSize,
                        color: color,
                      ),
                    ),
                  ),
                  const SizedBox(height: gap),
                  Text(
                    moodLabels[index],
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    textAlign: TextAlign.center,
                    style: const TextStyle(
                      fontSize: labelFont,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ],
              ),
            );
          }

          return Padding(
            padding: const EdgeInsets.only(top: 10),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: List.generate(4, item),
            ),
          );
        },
      ),
    );
  }
}
