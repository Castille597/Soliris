import 'package:flutter/material.dart';

class WifiSettingsPage extends StatefulWidget {
  const WifiSettingsPage({super.key});

  @override
  State<WifiSettingsPage> createState() => _WifiSettingsPageState();
}

class _WifiSettingsPageState extends State<WifiSettingsPage> {
  bool wifiEnabled = true;
  String? selectedNetwork;

  final List<Map<String, dynamic>> wifiNetworks = const [
    {'name': 'Soliris-Home', 'secured': true},
    {'name': 'Sunshine-WiFi ☀️', 'secured': false},
    {'name': 'Breathe-Easy 🌿', 'secured': true},
    {'name': 'PumpkinSpice-Net 🎃', 'secured': false},
    {'name': 'CloudNine ☁️', 'secured': true},
    {'name': 'Starlight-Link ✨', 'secured': false},
    {'name': 'CareBear-Net 💖', 'secured': true},
  ];

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final t = MediaQuery.of(context).textScaleFactor.clamp(1.0, 1.15);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      appBar: AppBar(
        backgroundColor: const Color(0xFFFF9800),
        elevation: 0,
        centerTitle: true,
        iconTheme: const IconThemeData(color: Colors.white),
        actionsIconTheme: const IconThemeData(color: Colors.white),
        leading: IconButton(
          tooltip: 'Back',
          iconSize: 22 * t,
          icon: const Icon(
            Icons.arrow_back_ios_new_rounded,
            color: Colors.white,
          ),
          onPressed: () => Navigator.of(context).maybePop(),
        ),
        title: const Text(
          'Wi-Fi Settings',
          style: TextStyle(
            fontSize: 22,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
      ),
      body: Column(
        children: [
          SwitchListTile(
            title: Text(
              'Turn on Wi-Fi',
              style: TextStyle(color: theme.textTheme.bodyMedium?.color),
            ),
            value: wifiEnabled,
            activeColor: Colors.orange,
            onChanged: (v) => setState(() => wifiEnabled = v),
          ),
          const Divider(thickness: 1, height: 0),
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 10, horizontal: 16),
            child: Align(
              alignment: Alignment.centerLeft,
              child: Text(
                'Available networks',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: theme.textTheme.bodyMedium?.color,
                ),
              ),
            ),
          ),
          Expanded(
            child: AbsorbPointer(
              absorbing: !wifiEnabled,
              child: AnimatedOpacity(
                opacity: wifiEnabled ? 1.0 : 0.5,
                duration: const Duration(milliseconds: 200),
                child: ListView.builder(
                  itemCount: wifiNetworks.length,
                  itemBuilder: (context, index) {
                    final network = wifiNetworks[index];
                    final String name = network['name'] as String;
                    final bool secured = network['secured'] as bool;
                    final bool isSelected = selectedNetwork == name;

                    return GestureDetector(
                      onTap: () => setState(() => selectedNetwork = name),
                      child: Container(
                        margin: const EdgeInsets.symmetric(
                          horizontal: 16,
                          vertical: 6,
                        ),
                        padding: const EdgeInsets.symmetric(
                          horizontal: 16,
                          vertical: 12,
                        ),
                        decoration: BoxDecoration(
                          color: isSelected
                              ? Colors.orange.withOpacity(0.1)
                              : theme.cardColor,
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Row(
                          children: [
                            Icon(
                              isSelected
                                  ? Icons.check_circle
                                  : Icons.radio_button_unchecked,
                              color: isSelected ? Colors.orange : Colors.grey,
                            ),
                            const SizedBox(width: 16),
                            Expanded(
                              child: Text(
                                name,
                                overflow: TextOverflow.ellipsis,
                                style: TextStyle(
                                  fontSize: 16,
                                  fontWeight: FontWeight.w500,
                                  color: theme.textTheme.bodyMedium?.color,
                                ),
                              ),
                            ),
                            if (secured)
                              const Icon(
                                Icons.lock,
                                size: 20,
                                color: Colors.grey,
                              ),
                            const SizedBox(width: 10),
                            const Icon(
                              Icons.wifi,
                              size: 22,
                              color: Colors.orange,
                            ),
                          ],
                        ),
                      ),
                    );
                  },
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
