import 'package:flutter/material.dart';
import '../device_ctrl.dart';

class LedVibrationsPage extends StatefulWidget {
  const LedVibrationsPage({super.key});
  @override
  State<LedVibrationsPage> createState() => _LedVibrationsPageState();
}

class _LedVibrationsPageState extends State<LedVibrationsPage> {
  bool ledEnabled = true;
  bool vibrationEnabled = true;

  final List<Map<String, String>> alertData = const [
    {
      'condition': 'Hydration Alert',
      'led': 'Blue',
      'vibration': '1 Vibration slow (buzzer)',
      'kind': 'hydr',
    },
    {
      'condition': 'Sun',
      'led': 'Orange',
      'vibration': '1 Vibration long (buzzer)',
      'kind': 'sun',
    },
    {
      'condition': 'Polluted Air',
      'led': 'Yellow',
      'vibration': '2 Vibrations quick (buzzer)',
      'kind': 'air',
    },
    {
      'condition': 'Fall or Fainting',
      'led': 'Red',
      'vibration': 'Long noise from buzzer',
      'kind': 'fall',
    },
    {
      'condition': 'Heart beating too fast / Intense activity',
      'led': 'Red',
      'vibration': '2 Vibrations (buzzer) slow',
      'kind': 'hrt',
    },
  ];

  Future<void> _safeSend(Map<String, dynamic> j) async {
    try {
      await deviceCtrl(j);
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Sent: ${j.keys.first} = ${j.values.first}'),
          duration: const Duration(milliseconds: 900),
        ),
      );
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Error: $e'),
          duration: const Duration(seconds: 2),
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      appBar: AppBar(
        backgroundColor: const Color(0xFFFF9800),
        elevation: 0,
        centerTitle: true,
        title: const Text(
          'LED / Vibrations',
          style: TextStyle(
            fontSize: 22,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            SwitchListTile(
              title: Text(
                'Turn on LEDs',
                style: TextStyle(color: theme.textTheme.bodyMedium?.color),
              ),
              value: ledEnabled,
              activeColor: Colors.orange,
              onChanged: (value) {
                setState(() => ledEnabled = value);
                _safeSend({'led': value});
              },
            ),

            SwitchListTile(
              title: Text(
                'Turn on the Buzzer',
                style: TextStyle(color: theme.textTheme.bodyMedium?.color),
              ),
              value: vibrationEnabled,
              activeColor: Colors.orange,
              onChanged: (value) {
                setState(() => vibrationEnabled = value);
                _safeSend({'buzz': value});
              },
            ),

            const SizedBox(height: 16),
            Text(
              'Signal Codes',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
                color: theme.textTheme.bodyMedium?.color,
              ),
            ),
            const SizedBox(height: 10),

            Expanded(
              child: ListView.builder(
                itemCount: alertData.length,
                itemBuilder: (context, index) {
                  final item = alertData[index];
                  return InkWell(
                    borderRadius: BorderRadius.circular(20),
                    onTap: () => _safeSend({'play': item['kind']!}),
                    child: Container(
                      margin: const EdgeInsets.only(bottom: 12),
                      padding: const EdgeInsets.all(16),
                      decoration: BoxDecoration(
                        color: theme.cardColor,
                        borderRadius: BorderRadius.circular(20),
                        border: Border.all(
                          color: Colors.orange.withOpacity(0.15),
                        ),
                      ),
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  item['condition']!,
                                  style: TextStyle(
                                    fontWeight: FontWeight.bold,
                                    fontSize: 16,
                                    color: theme.textTheme.bodyMedium?.color,
                                  ),
                                ),
                                const SizedBox(height: 6),
                                Text(
                                  'LED : ${item['led']}',
                                  style: TextStyle(
                                    color: theme.textTheme.bodyMedium?.color,
                                  ),
                                ),
                                Text(
                                  'Vibration : ${item['vibration']}',
                                  style: TextStyle(
                                    color: theme.textTheme.bodyMedium?.color,
                                  ),
                                ),
                              ],
                            ),
                          ),
                          const SizedBox(width: 12),
                          OutlinedButton(
                            onPressed: () => _safeSend({'play': item['kind']!}),
                            child: const Text('Try'),
                          ),
                        ],
                      ),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}
