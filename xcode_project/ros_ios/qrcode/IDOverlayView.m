//
//  IDOverlayView.m
//  Barcodes_iOS7_Sample
//
//  Created by Иван Джеферов on 10/12/13.
//  Copyright (c) 2013 Incept Development. All rights reserved.
//

#import "IDOverlayView.h"

@interface IDOverlayView ()

@property (nonatomic, strong) CAShapeLayer *outlineLayer;

- (UIBezierPath *)createPathFromPoints:(NSArray *)points;

@end

@implementation IDOverlayView

#pragma mark - Init

- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];

    if (self)
    {
        self.outlineLayer = [[CAShapeLayer alloc] init];
        self.outlineLayer.strokeColor = [[[UIColor yellowColor] colorWithAlphaComponent:0.8] CGColor];
        self.outlineLayer.lineWidth = 3.0;
        self.outlineLayer.fillColor = [[UIColor clearColor] CGColor];
        [self.layer addSublayer:self.outlineLayer];
    }

    return self;
}

#pragma mark - Property accessors

@synthesize corners = _corners;

- (void)setCorners:(NSArray *)corners
{
    if (corners != _corners)
    {
        _corners = corners;
        UIBezierPath *outlinePath = [self createPathFromPoints:corners];
        self.outlineLayer.path = [outlinePath CGPath];
    }
}

#pragma mark - Private

- (UIBezierPath *)createPathFromPoints:(NSArray *)points
{
    UIBezierPath *path = [UIBezierPath new];

    // Start at the first corner
    [path moveToPoint:[[points firstObject] CGPointValue]];

    // Now draw lines around the corners
    for (NSUInteger i = 1; i < [points count]; i++)
    {
        [path addLineToPoint:[points[i] CGPointValue]];
    }

    // And join it back to the first corner
    [path addLineToPoint:[[points firstObject] CGPointValue]];

    return path;
}

@end
