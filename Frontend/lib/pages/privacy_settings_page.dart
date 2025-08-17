import 'package:flutter/material.dart';
import '../theme/scale_utils.dart';

class PrivacySettingsPage extends StatefulWidget {
  const PrivacySettingsPage({super.key});

  @override
  State<PrivacySettingsPage> createState() => _PrivacySettingsPageState();
}

class _PrivacySettingsPageState extends State<PrivacySettingsPage> {
  bool? consentGiven;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;

    final double arrowSize =
        (22.0 * MediaQuery.of(context).textScaleFactor.clamp(1.0, 1.15));

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      appBar: AppBar(
        backgroundColor: cs.primary,
        elevation: 0,
        centerTitle: true,
        foregroundColor: cs.onPrimary,
        iconTheme: IconThemeData(color: cs.onPrimary),

        leading: IconButton(
          tooltip: 'Back',
          iconSize: arrowSize,
          icon: Icon(Icons.arrow_back_ios_new_rounded, color: cs.onPrimary),
          onPressed: () => Navigator.of(context).maybePop(),
        ),
        title: Text(
          'Privacy Settings',
          style: TextStyle(color: cs.onPrimary, fontWeight: FontWeight.w800),
        ),
      ),
      body: SafeArea(
        child: SingleChildScrollView(
          padding: EdgeInsets.all(20.sx(context)),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Center(
                child: Icon(
                  Icons.lock_outline,
                  size: 70.sx(context),
                  color: cs.primary,
                ),
              ),
              SizedBox(height: 20.sx(context)),

              Text(
                "Your health, your privacy.",
                textAlign: TextAlign.center,
                style: theme.textTheme.titleLarge?.copyWith(
                  fontWeight: FontWeight.bold,
                ),
              ),
              SizedBox(height: 20.sx(context)),

              const _Bullet(
                "All data is fully anonymized before any analysis; your identity cannot be traced.",
              ),
              const _Bullet(
                "No personally identifiable information (PII) is ever stored, shared, or linked back to you.",
              ),
              const _Bullet(
                "Data may be shared anonymously with certified medical professionals and community platforms for healthcare improvement and research.",
              ),
              const _Bullet(
                "We comply with strict privacy and security standards (e.g., GDPR and HIPAA-aligned practices).",
              ),
              const _Bullet(
                "You remain in control. You can withdraw consent at any time.",
              ),

              SizedBox(height: 24.sx(context)),

              Wrap(
                alignment: WrapAlignment.center,
                spacing: 12.sx(context),
                runSpacing: 12.sx(context),
                children: [
                  ElevatedButton.icon(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      foregroundColor: Colors.white,
                      padding: EdgeInsets.symmetric(
                        horizontal: 20.sx(context),
                        vertical: 12.sx(context),
                      ),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12.sx(context)),
                      ),
                      elevation: 0,
                    ),
                    onPressed: () {
                      setState(() => consentGiven = true);
                      Navigator.pop(context);
                    },
                    icon: Icon(Icons.check, size: 20.sx(context)),
                    label: Text(
                      'Yes, I confirm',
                      textScaleFactor: 1.0,
                      style: theme.textTheme.titleSmall?.copyWith(
                        color: Colors.white,
                        fontWeight: FontWeight.w700,
                      ),
                    ),
                  ),
                  ElevatedButton.icon(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.red,
                      foregroundColor: Colors.white,
                      padding: EdgeInsets.symmetric(
                        horizontal: 20.sx(context),
                        vertical: 12.sx(context),
                      ),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12.sx(context)),
                      ),
                      elevation: 0,
                    ),
                    onPressed: () {
                      setState(() => consentGiven = false);
                      Navigator.pop(context);
                    },
                    icon: Icon(Icons.close, size: 20.sx(context)),
                    label: Text(
                      'No',
                      textScaleFactor: 1.0,
                      style: theme.textTheme.titleSmall?.copyWith(
                        color: Colors.white,
                        fontWeight: FontWeight.w700,
                      ),
                    ),
                  ),
                ],
              ),
              SizedBox(height: 12.sx(context)),
            ],
          ),
        ),
      ),
    );
  }
}

class _Bullet extends StatelessWidget {
  const _Bullet(this.text);
  final String text;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Padding(
      padding: EdgeInsets.only(bottom: 10.sx(context)),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Padding(
            padding: EdgeInsets.only(top: 7.sx(context), right: 10.sx(context)),
            child: Container(
              width: 8.sx(context),
              height: 8.sx(context),
              decoration: BoxDecoration(
                color: theme.colorScheme.onSurface,
                shape: BoxShape.circle,
              ),
            ),
          ),

          Expanded(child: Text(text, style: theme.textTheme.bodyMedium)),
        ],
      ),
    );
  }
}
