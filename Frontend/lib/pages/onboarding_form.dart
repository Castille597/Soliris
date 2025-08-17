import 'package:flutter/material.dart';
import 'home_navigator.dart';
import '../theme/theme_controller.dart';
import '../profile_store.dart';

const List<String> _genderItems = ['Woman', 'Man', 'Other'];
String _genderKeyFromDisplay(String d) => d.toLowerCase();
String _genderDisplayFromKey(String k) {
  switch (k.toLowerCase()) {
    case 'woman':
      return 'Woman';
    case 'man':
      return 'Man';
    default:
      return 'Other';
  }
}

class OnboardingFormPage extends StatefulWidget {
  const OnboardingFormPage({super.key});

  @override
  State<OnboardingFormPage> createState() => _OnboardingFormPageState();
}

class _OnboardingFormPageState extends State<OnboardingFormPage> {
  final TextEditingController nameController = TextEditingController();
  final TextEditingController ageController = TextEditingController();
  final TextEditingController emergencyPhoneController =
      TextEditingController();

  String selectedGenderDisplay = 'Woman';

  @override
  void initState() {
    super.initState();
    final p = ProfileStore.instance.profile.value;
    if (p != null) {
      nameController.text = p.displayName;
      selectedGenderDisplay = _genderDisplayFromKey(p.gender);
      if (p.age != null) ageController.text = p.age.toString();
      emergencyPhoneController.text = p.emergencyPhone ?? '';
      setState(() {});
    }
  }

  @override
  void dispose() {
    nameController.dispose();
    ageController.dispose();
    emergencyPhoneController.dispose();
    super.dispose();
  }

  Widget _field(
    String label,
    TextEditingController c, {
    TextInputType type = TextInputType.text,
  }) {
    final theme = Theme.of(context);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          label,
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: theme.textTheme.bodyMedium?.color,
          ),
        ),
        const SizedBox(height: 6),
        TextField(
          controller: c,
          keyboardType: type,
          style: TextStyle(color: theme.textTheme.bodyMedium?.color),
          decoration: InputDecoration(
            filled: true,
            fillColor: theme.cardColor,
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(12),
              borderSide: BorderSide.none,
            ),
          ),
        ),
        const SizedBox(height: 16),
      ],
    );
  }

  Future<void> _continue() async {
    final name = nameController.text.trim();
    final int? age = int.tryParse(ageController.text.trim());
    final emergencyPhone = emergencyPhoneController.text.trim();

    if (name.isEmpty) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('Please enter your name')));
      return;
    }

    final profile = UserProfile(
      displayName: name,
      gender: _genderKeyFromDisplay(selectedGenderDisplay),
      age: age,
      emergencyPhone: emergencyPhone.isEmpty ? null : emergencyPhone,
    );

    await ProfileStore.instance.save(profile);

    if (!mounted) return;
    Navigator.pushReplacement(
      context,
      MaterialPageRoute(builder: (_) => const HomeNavigator()),
    );
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      appBar: AppBar(
        backgroundColor: Colors.orange,
        title: const Text('Create your profile'),
        centerTitle: true,
      ),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: ListView(
          children: [
            const SizedBox(height: 20),
            _field("Name", nameController),

            Text(
              'Gender',
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: theme.textTheme.bodyMedium?.color,
              ),
            ),
            const SizedBox(height: 6),
            DropdownButtonFormField<String>(
              value: _genderItems.contains(selectedGenderDisplay)
                  ? selectedGenderDisplay
                  : null,
              items: _genderItems
                  .map((e) => DropdownMenuItem(value: e, child: Text(e)))
                  .toList(),
              onChanged: (v) =>
                  setState(() => selectedGenderDisplay = v ?? 'Woman'),
              style: TextStyle(color: theme.textTheme.bodyMedium?.color),
              decoration: InputDecoration(
                filled: true,
                fillColor: theme.cardColor,
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                  borderSide: BorderSide.none,
                ),
              ),
            ),
            const SizedBox(height: 16),

            _field("Age", ageController, type: TextInputType.number),
            _field(
              "Emergency contact (phone)",
              emergencyPhoneController,
              type: TextInputType.phone,
            ),

            const SizedBox(height: 30),
            ElevatedButton(
              onPressed: _continue,
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.orange,
                padding: const EdgeInsets.symmetric(vertical: 16),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
              child: const Text(
                'Continue',
                style: TextStyle(fontSize: 16, color: Colors.white),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
