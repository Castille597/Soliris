import 'package:flutter/material.dart';
import '../profile_store.dart';
import '../theme/scale_utils.dart';

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

String? _nz(String s) {
  final t = s.trim();
  return t.isEmpty ? null : t;
}

class EditProfilePage extends StatefulWidget {
  const EditProfilePage({super.key});

  @override
  State<EditProfilePage> createState() => _EditProfilePageState();
}

class _EditProfilePageState extends State<EditProfilePage> {
  final TextEditingController firstNameController = TextEditingController();
  final TextEditingController userPhoneController = TextEditingController();
  final TextEditingController ageController = TextEditingController();

  final TextEditingController emergencyNameController = TextEditingController();
  final TextEditingController emergencyPhoneController =
      TextEditingController();

  final TextEditingController doctorNameController = TextEditingController();
  final TextEditingController doctorPhoneController = TextEditingController();

  String selectedGenderDisplay = 'Woman';

  @override
  void initState() {
    super.initState();
    final p = ProfileStore.instance.profile.value;
    if (p != null) {
      firstNameController.text = p.displayName;
      selectedGenderDisplay = _genderDisplayFromKey(p.gender);
      if (p.age != null) ageController.text = p.age.toString();

      userPhoneController.text = p.userPhone ?? '';
      emergencyNameController.text = p.emergencyName ?? '';
      emergencyPhoneController.text = p.emergencyPhone ?? '';
      doctorNameController.text = p.doctorName ?? '';
      doctorPhoneController.text = p.doctorPhone ?? '';
      setState(() {});
    }
  }

  @override
  void dispose() {
    firstNameController.dispose();
    userPhoneController.dispose();
    ageController.dispose();
    emergencyNameController.dispose();
    emergencyPhoneController.dispose();
    doctorNameController.dispose();
    doctorPhoneController.dispose();
    super.dispose();
  }

  Widget _label(BuildContext context, String s) => Padding(
    padding: EdgeInsets.only(bottom: 6.sx(context)),
    child: Text(
      s,
      style: Theme.of(
        context,
      ).textTheme.bodyMedium?.copyWith(fontWeight: FontWeight.bold),
    ),
  );

  Widget _field(
    BuildContext context,
    TextEditingController c, {
    TextInputType type = TextInputType.text,
  }) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;
    return TextField(
      controller: c,
      keyboardType: type,
      style: TextStyle(color: theme.textTheme.bodyMedium?.color),
      decoration: InputDecoration(
        contentPadding: EdgeInsets.symmetric(
          horizontal: 12.sx(context),
          vertical: 12.sx(context),
        ),
        filled: true,
        fillColor: theme.cardColor,
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12.sx(context)),
          borderSide: BorderSide.none,
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12.sx(context)),
          borderSide: BorderSide(color: cs.primary.withOpacity(.4), width: 1),
        ),
      ),
    );
  }

  Future<void> _save() async {
    final int? age = int.tryParse(ageController.text.trim());
    final p0 = ProfileStore.instance.profile.value;

    final updated = (p0 ?? const UserProfile(displayName: '', gender: 'other'))
        .copyWith(
          displayName: firstNameController.text.trim(),
          gender: _genderKeyFromDisplay(selectedGenderDisplay),
          age: age,
          userPhone: _nz(userPhoneController.text),
          emergencyName: _nz(emergencyNameController.text),
          emergencyPhone: _nz(emergencyPhoneController.text),
          doctorName: _nz(doctorNameController.text),
          doctorPhone: _nz(doctorPhoneController.text),
        );

    await ProfileStore.instance.save(updated);
    if (!mounted) return;
    Navigator.pop(context, true);
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
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
          'Profile Settings',
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
      body: Padding(
        padding: EdgeInsets.all(20.sx(context)),
        child: ListView(
          children: [
            Text(
              'Personal Information',
              style: theme.textTheme.titleMedium?.copyWith(
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 10.sx(context)),

            _label(context, 'Name'),
            _field(context, firstNameController),
            SizedBox(height: 12.sx(context)),

            _label(context, 'Phone Number'),
            _field(context, userPhoneController, type: TextInputType.phone),
            SizedBox(height: 12.sx(context)),

            _label(context, 'Gender'),
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
                contentPadding: EdgeInsets.symmetric(
                  horizontal: 12.sx(context),
                  vertical: 12.sx(context),
                ),
                filled: true,
                fillColor: theme.cardColor,
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12.sx(context)),
                  borderSide: BorderSide.none,
                ),
              ),
            ),
            SizedBox(height: 12.sx(context)),

            _label(context, 'Age'),
            _field(context, ageController, type: TextInputType.number),
            SizedBox(height: 20.sx(context)),

            Text(
              'Emergency Contact',
              style: theme.textTheme.titleMedium?.copyWith(
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 10.sx(context)),

            _label(context, 'Name of Contact'),
            _field(context, emergencyNameController),
            SizedBox(height: 12.sx(context)),

            _label(context, 'Phone Number'),
            _field(
              context,
              emergencyPhoneController,
              type: TextInputType.phone,
            ),
            SizedBox(height: 20.sx(context)),

            Text(
              'Attending Physician',
              style: theme.textTheme.titleMedium?.copyWith(
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 10.sx(context)),

            _label(context, 'Name of the Physician'),
            _field(context, doctorNameController),
            SizedBox(height: 12.sx(context)),

            _label(context, 'Phone Number'),
            _field(context, doctorPhoneController, type: TextInputType.phone),

            SizedBox(height: 24.sx(context)),
            ElevatedButton(
              onPressed: _save,
              style: ElevatedButton.styleFrom(
                backgroundColor: cs.primary,
                foregroundColor: cs.onPrimary,
                padding: EdgeInsets.symmetric(vertical: 16.sx(context)),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12.sx(context)),
                ),
                elevation: 0,
              ),
              child: Text(
                'Save',
                style: theme.textTheme.titleMedium?.copyWith(
                  color: cs.onPrimary,
                  fontWeight: FontWeight.w700,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
