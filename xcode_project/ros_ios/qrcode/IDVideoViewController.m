//
//  IDVideoViewController.m
//  Barcodes_iOS7_Sample
//
//  Created by Иван Джеферов on 10/12/13.
//  Copyright (c) 2013 Incept Development. All rights reserved.
//

#import "IDVideoViewController.h"
#import "IDCodeScanner.h"
#import "IDOverlayView.h"

@interface IDVideoViewController ()

// UI
@property (nonatomic, strong) UIView *cameraView;
@property (nonatomic, strong) UISwitch *changeCameraSwitch;
@property (nonatomic, strong) UILabel *textLabel;
@property (nonatomic, strong) UISwitch *torchSwitch;
@property (nonatomic, strong) IDOverlayView *overlayView;

// The code scanner instance
@property (nonatomic, strong) IDCodeScanner *codeScanner;

// Actions
- (void)changeTorchState:(UISwitch *)sender;

@end

@implementation IDVideoViewController

#pragma mark - View Controller

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.navigationItem.leftBarButtonItem=[[UIBarButtonItem alloc] initWithTitle:@"Configuration" style:UIBarButtonItemStylePlain target:self action:@selector(backToIndex)];
    

    // Block object of self
    __weak IDVideoViewController *videoVC = self;
    
    // Init code scanner
    self.codeScanner = [[IDCodeScanner alloc] init];
    [self.codeScanner setDidFailWithError:^(IDCodeScanner *cs, NSError *error)
    {
        NSLog(@"Code scanner :%@ error: %@", cs, error);
    }];
    [self.codeScanner setShouldAddLayerFromCaptureSessionToView:^(IDCodeScanner *cs, AVCaptureDevicePosition capturePosition, AVCaptureSession *avcs, AVCaptureVideoPreviewLayer *avcvpl)
    {
        avcvpl.backgroundColor = [UIColor clearColor].CGColor;
        avcvpl.frame = CGRectMake(0, 0, videoVC.view.frame.size.width, videoVC.view.frame.size.height - 44.0f);
        [videoVC.view.layer addSublayer:avcvpl];
    }];
    [self.codeScanner setDidRecognizeMetadataObject:^(IDCodeScanner *cs, AVCaptureOutput *captureOutput, AVCaptureConnection *captureConnection, AVCaptureVideoPreviewLayer *previewLayer, AVMetadataMachineReadableCodeObject *metadataObject)
    {
        NSLog(@"Recognized metadata object: %@", metadataObject);

        if ([metadataObject isKindOfClass:[AVMetadataFaceObject class]])
        {
            // Tells you to smile
            videoVC.textLabel.text = @"Smile";
        }
        else
        {
            // Show text
            videoVC.textLabel.text = metadataObject.stringValue;
            
            // Show overlay
            AVMetadataMachineReadableCodeObject *transformed = (AVMetadataMachineReadableCodeObject *)[previewLayer transformedMetadataObjectForMetadataObject:metadataObject];
            
            // Update the frame on the _boundingBox view, and show it
            videoVC.overlayView.frame = transformed.bounds;
            videoVC.overlayView.hidden = NO;
            [videoVC.view bringSubviewToFront:videoVC.overlayView];
            
            // Now convert the corners array into CGPoints in the coordinate system
            //  of the bounding box itself
            NSArray *translatedCorners = [videoVC translatePoints:transformed.corners
                                                         fromView:videoVC.view
                                                           toView:videoVC.overlayView];

            // Set the corners array
            videoVC.overlayView.corners = translatedCorners;
            
            double delayInSeconds = 1.0;
            dispatch_time_t popTime = dispatch_time(DISPATCH_TIME_NOW, (int64_t)(delayInSeconds * NSEC_PER_SEC));
            dispatch_after(popTime, dispatch_get_main_queue(), ^(void)
            {
                videoVC.overlayView.hidden = YES;
            });
        }
    }];

    // Set metadata types to recognize
    NSArray *metadataObjectTypes = @[AVMetadataObjectTypeQRCode,
                                     AVMetadataObjectTypePDF417Code,
                                     AVMetadataObjectTypeCode128Code,
                                     AVMetadataObjectTypeCode93Code,
                                     AVMetadataObjectTypeEAN8Code,
                                     AVMetadataObjectTypeEAN13Code,
                                     AVMetadataObjectTypeCode39Mod43Code,
                                     AVMetadataObjectTypeCode39Code,
                                     AVMetadataObjectTypeUPCECode,
                                     AVMetadataObjectTypeFace];

    // Start capture session for front camera
    //[self.codeScanner startCaptureSessionForCapturePosition:AVCaptureDevicePositionFront andMetadataObjectTypes:metadataObjectTypes];

    // Start capture session for back camera
    [self.codeScanner startCaptureSessionForCapturePosition:AVCaptureDevicePositionBack andMetadataObjectTypes:metadataObjectTypes];

    // Add button to switch camera
    self.changeCameraSwitch = [[UISwitch alloc] init];
    [self.changeCameraSwitch addTarget:self.codeScanner action:@selector(switchCamera) forControlEvents:UIControlEventValueChanged];
    self.changeCameraSwitch.backgroundColor = [UIColor clearColor];
    self.changeCameraSwitch.on = YES;
    [self.changeCameraSwitch sizeToFit];
    self.changeCameraSwitch.frame = CGRectMake(7.0f,
                                               self.view.frame.size.height - self.changeCameraSwitch.frame.size.height - 7.0f,
                                               self.changeCameraSwitch.frame.size.width,
                                               self.changeCameraSwitch.frame.size.height);
    [self.view addSubview:self.changeCameraSwitch];

    // Torch switch
    self.torchSwitch = [[UISwitch alloc] init];
    [self.torchSwitch addTarget:self action:@selector(changeTorchState:) forControlEvents:UIControlEventValueChanged];
    self.torchSwitch.backgroundColor = [UIColor clearColor];
    self.torchSwitch.on = self.codeScanner.torchOn;
    [self.torchSwitch sizeToFit];
    self.torchSwitch.frame = CGRectMake(self.view.frame.size.width - self.torchSwitch.frame.size.width - 7.0f,
                                        self.view.frame.size.height - self.torchSwitch.frame.size.height - 7.0f,
                                        self.torchSwitch.frame.size.width,
                                        self.torchSwitch.frame.size.height);
    [self.view addSubview:self.torchSwitch];
    
    // Text label
    self.textLabel = [[UILabel alloc] init];
    self.textLabel.backgroundColor = [UIColor whiteColor];
    self.textLabel.textAlignment = NSTextAlignmentCenter;
    self.textLabel.textColor = [UIColor lightGrayColor];
    self.textLabel.font = [UIFont fontWithName:@"Helvetica" size:22.0f];
    self.textLabel.adjustsFontSizeToFitWidth = YES;
    self.textLabel.frame = CGRectMake(self.changeCameraSwitch.frame.origin.x + self.changeCameraSwitch.frame.size.width,
                                      self.view.frame.size.height - 44.0f,
                                      self.view.frame.size.width - self.changeCameraSwitch.frame.size.width - self.torchSwitch.frame.size.width - 14.0f,
                                      44.0f);
    [self.view addSubview:self.textLabel];
    
    // The rectangle which outlines a recognized metadata object
    self.overlayView = [[IDOverlayView alloc] init];
    self.overlayView.backgroundColor = [UIColor clearColor];
    self.overlayView.hidden = YES;
    [self.view addSubview:self.overlayView];
}

#pragma mark - Rotation

- (BOOL)shouldAutorotate
{
    return NO;
}

- (NSUInteger)supportedInterfaceOrientations
{
    return UIInterfaceOrientationMaskPortrait;
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)toInterfaceOrientation
{
    return UIInterfaceOrientationIsPortrait(toInterfaceOrientation);
}

#pragma mark - Actions

- (void)changeTorchState:(UISwitch *)sender
{
    if ([sender isKindOfClass:[UISwitch class]])
    {
        self.codeScanner.torchOn = sender.isOn;
    }
}

#pragma mark - Private

- (NSArray *)translatePoints:(NSArray *)points fromView:(UIView *)fromView toView:(UIView *)toView
{
    NSMutableArray *translatedPoints = [NSMutableArray new];

    // The points are provided in a dictionary with keys X and Y
    for (NSDictionary *point in points)
    {
        // Let's turn them into CGPoints
        CGPoint pointValue = CGPointMake([point[@"X"] floatValue], [point[@"Y"] floatValue]);
        // Now translate from one view to the other
        CGPoint translatedPoint = [fromView convertPoint:pointValue toView:toView];
        // Box them up and add to the array
        [translatedPoints addObject:[NSValue valueWithCGPoint:translatedPoint]];
    }

    return [translatedPoints copy];
}
-(void)backToIndex

{
    
    //do something.
    [[NSNotificationCenter defaultCenter] postNotificationName:@"ChangeLabelTextNotification" object:self.textLabel.text];
    NSLog(self.textLabel.text);
    [self.navigationController popViewControllerAnimated:YES];
    
}


@end
