import 'dart:math';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:flutter/services.dart';
import 'package:pdf/pdf.dart';
import 'package:pdf/widgets.dart' as pw;
import 'package:printing/printing.dart';

import '../widgets/multi_metric_chart.dart';
import '../widgets/notification_bell.dart';
import '../alert_center.dart';
import 'alert_page.dart';
import '../theme/scale_utils.dart';

class HealthDashboardPage extends StatefulWidget {
  const HealthDashboardPage({super.key});

  @override
  State<HealthDashboardPage> createState() => _HealthDashboardPageState();
}

class _HealthDashboardPageState extends State<HealthDashboardPage> {
  static const Color kTopBarOrange = Color(0xFFFF9800);
  static const Color kDeepOrange = Color.fromARGB(255, 212, 128, 1);
  static const Color kSoftBorder = Color(0xFFFFE0B2);

  DateTime selectedDate = DateTime.now();

  final Map<String, List<FlSpot>> _series = {
    'HR': <FlSpot>[],
    'SpO₂': <FlSpot>[],
    'Skin temp': <FlSpot>[],
    'Resp rate': <FlSpot>[],
    'Steps/min': <FlSpot>[],
  };

  final Map<String, bool> _visible = {
    'HR': true,
    'SpO₂': true,
    'Skin temp': true,
    'Resp rate': false,
    'Steps/min': false,
  };

  @override
  void initState() {
    super.initState();
    _rebuildSeriesFor(selectedDate);
  }

  void _rebuildSeriesFor(DateTime day) {
    final seed = DateTime(day.year, day.month, day.day).millisecondsSinceEpoch;
    final rnd = Random(seed);

    final hr = <FlSpot>[];
    final sp = <FlSpot>[];
    final sk = <FlSpot>[];
    final rr = <FlSpot>[];
    final steps = <FlSpot>[];

    for (int m = 0; m <= 1440; m += 15) {
      final t = m / 1440.0 * 2 * pi;
      final hrVal = 65 + 8 * sin(t * 2) + rnd.nextDouble() * 6;
      final spVal = 97 + 1.0 * sin(t) + rnd.nextDouble() * 0.6;
      final skVal = 36.3 + 0.6 * sin(t) + rnd.nextDouble() * 0.2;
      final rrVal = 14 + 2.0 * sin(t * 1.5) + rnd.nextDouble();
      final dayBoost = max(0.0, sin(t - pi / 2));
      final stepsVal = max(
        0.0,
        10 + 40 * dayBoost + 20 * sin(t * 3) + rnd.nextDouble() * 10,
      );

      hr.add(FlSpot(m.toDouble(), hrVal));
      sp.add(FlSpot(m.toDouble(), spVal));
      sk.add(FlSpot(m.toDouble(), skVal));
      rr.add(FlSpot(m.toDouble(), rrVal));
      steps.add(FlSpot(m.toDouble(), stepsVal));
    }

    setState(() {
      _series['HR'] = hr;
      _series['SpO₂'] = sp;
      _series['Skin temp'] = sk;
      _series['Resp rate'] = rr;
      _series['Steps/min'] = steps;
    });
  }

  Future<void> generateHealthSummaryPDF(BuildContext context) async {
    final pdf = pw.Document();
    final ByteData bytes = await rootBundle.load('assets/images/unnamed.jpg');
    final img = pw.MemoryImage(bytes.buffer.asUint8List());

    pdf.addPage(
      pw.Page(
        pageFormat: PdfPageFormat.a4,
        build: (_) => pw.Column(
          crossAxisAlignment: pw.CrossAxisAlignment.start,
          children: [
            pw.Row(
              mainAxisAlignment: pw.MainAxisAlignment.spaceBetween,
              children: [
                pw.Text(
                  'Soliris Health Summary',
                  style: pw.TextStyle(
                    fontSize: 22,
                    fontWeight: pw.FontWeight.bold,
                  ),
                ),
                pw.Image(img, width: 36),
              ],
            ),
            pw.SizedBox(height: 6),
            pw.Text(
              'Date: ${selectedDate.toLocal().toString().split(" ").first}',
            ),
            pw.SizedBox(height: 16),
            pw.Text(
              'Metrics: HR, SpO₂, Skin temp, Resp rate, Steps/min (sample)',
            ),
            pw.SizedBox(height: 24),
            pw.Text('This is a placeholder PDF. Add charts/tables as needed.'),
          ],
        ),
      ),
    );

    await Printing.layoutPdf(onLayout: (_) async => pdf.save());
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final t = MediaQuery.of(context).textScaleFactor.clamp(1.0, 1.15);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      appBar: AppBar(
        backgroundColor: kTopBarOrange,
        centerTitle: true,
        elevation: 0,
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
          'Health Documentation',
          style: TextStyle(
            color: Colors.white,
            fontSize: 20,
            fontWeight: FontWeight.w800,
          ),
        ),
        actions: [
          NotificationBell(
            color: Colors.white,
            onPressed: () async {
              await Navigator.of(
                context,
              ).push(MaterialPageRoute(builder: (_) => const AlertPage()));
              AlertCenter.instance.markAllRead();
            },
          ),
          const SizedBox(width: 8),
        ],
      ),

      body: SingleChildScrollView(
        padding: EdgeInsets.all(16.sx(context)),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            ElevatedButton.icon(
              icon: const Icon(Icons.calendar_today, size: 18),
              label: Text(
                selectedDate.toLocal().toString().split(' ').first,
                style: const TextStyle(fontWeight: FontWeight.w800),
              ),
              style: ElevatedButton.styleFrom(
                backgroundColor: kDeepOrange,
                foregroundColor: Colors.white,
                padding: EdgeInsets.symmetric(
                  horizontal: 16.sx(context),
                  vertical: 10.sx(context),
                ),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(20.sx(context)),
                ),
                elevation: 0,
              ),
              onPressed: () async {
                final picked = await showDatePicker(
                  context: context,
                  initialDate: selectedDate,
                  firstDate: DateTime(2022),
                  lastDate: DateTime.now(),
                );
                if (picked != null) {
                  setState(() => selectedDate = picked);
                  _rebuildSeriesFor(picked);
                }
              },
            ),

            SizedBox(height: 16.sx(context)),

            Container(
              width: double.infinity,
              padding: EdgeInsets.all(12.sx(context)),
              decoration: BoxDecoration(
                color: theme.cardColor,
                borderRadius: BorderRadius.circular(20.sx(context)),
                border: Border.all(color: kSoftBorder),
              ),
              child: MultiMetricChart(
                series: _series,
                visible: _visible,
                onToggle: (k, sel) => setState(() => _visible[k] = sel),
              ),
            ),

            SizedBox(height: 24.sx(context)),

            Center(
              child: ElevatedButton.icon(
                icon: const Icon(Icons.picture_as_pdf),
                label: const Text(
                  'Export Health Summary',
                  style: TextStyle(fontWeight: FontWeight.bold),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: kDeepOrange,
                  foregroundColor: Colors.white,
                  padding: EdgeInsets.symmetric(
                    horizontal: 22.sx(context),
                    vertical: 14.sx(context),
                  ),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(16.sx(context)),
                  ),
                  elevation: 0,
                ),
                onPressed: () => generateHealthSummaryPDF(context),
              ),
            ),
            SizedBox(height: 12.sx(context)),
          ],
        ),
      ),
    );
  }
}
