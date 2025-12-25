"""
Monitoring and metrics collection service with performance monitoring and alerting
"""
import time
import logging
from typing import Dict, Any, Optional, Callable, List
from datetime import datetime
from dataclasses import dataclass
from enum import Enum
import threading
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# Configure logging
logger = logging.getLogger(__name__)


class AlertLevel(Enum):
    """
    Enum for alert levels
    """
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"


@dataclass
class Alert:
    """
    Data class for alert information
    """
    level: AlertLevel
    message: str
    timestamp: datetime
    source: str
    details: Optional[Dict[str, Any]] = None


@dataclass
class RequestMetrics:
    """
    Data class for request metrics
    """
    request_id: str
    endpoint: str
    method: str
    start_time: float
    end_time: Optional[float] = None
    response_time: Optional[float] = None
    status_code: Optional[int] = None
    error: Optional[str] = None

    def complete(self, status_code: int, error: Optional[str] = None):
        """
        Complete the metrics collection for this request

        Args:
            status_code: HTTP status code of the response
            error: Error message if any
        """
        self.end_time = time.time()
        self.response_time = self.end_time - self.start_time
        self.status_code = status_code
        self.error = error


class MonitoringService:
    """
    Service for monitoring and collecting metrics with performance monitoring and alerting
    """

    def __init__(self,
                 slow_request_threshold: float = 5.0,  # seconds
                 high_error_rate_threshold: float = 0.1,  # 10% error rate
                 high_response_time_threshold: float = 3.0,  # seconds
                 alert_callbacks: Optional[List[Callable]] = None):
        """
        Initialize the monitoring service

        Args:
            slow_request_threshold: Threshold for slow request alerts (in seconds)
            high_error_rate_threshold: Threshold for high error rate alerts (as decimal)
            high_response_time_threshold: Threshold for high response time alerts (in seconds)
            alert_callbacks: List of callback functions to call when alerts are triggered
        """
        self.request_metrics: Dict[str, RequestMetrics] = {}
        self.total_requests = 0
        self.failed_requests = 0
        self.start_time = time.time()

        # Alerting configuration
        self.slow_request_threshold = slow_request_threshold
        self.high_error_rate_threshold = high_error_rate_threshold
        self.high_response_time_threshold = high_response_time_threshold
        self.alert_callbacks = alert_callbacks or []
        self.alerts: List[Alert] = []

        # Performance thresholds
        self.performance_thresholds = {
            'response_time': high_response_time_threshold,
            'error_rate': high_error_rate_threshold,
            'slow_requests': slow_request_threshold
        }

        logger.info("MonitoringService initialized with performance monitoring and alerting")

    def start_request(self, request_id: str, endpoint: str, method: str) -> str:
        """
        Start tracking a request

        Args:
            request_id: Unique ID for the request
            endpoint: API endpoint being called
            method: HTTP method (GET, POST, etc.)

        Returns:
            Request ID for tracking
        """
        self.total_requests += 1
        metrics = RequestMetrics(
            request_id=request_id,
            endpoint=endpoint,
            method=method,
            start_time=time.time()
        )
        self.request_metrics[request_id] = metrics
        logger.debug(f"Started tracking request {request_id} to {method} {endpoint}")
        return request_id

    def complete_request(self, request_id: str, status_code: int, error: Optional[str] = None):
        """
        Complete tracking for a request

        Args:
            request_id: Request ID to complete tracking for
            status_code: HTTP status code of the response
            error: Error message if any
        """
        if request_id in self.request_metrics:
            metrics = self.request_metrics[request_id]
            metrics.complete(status_code, error)

            # Performance monitoring and alerting
            if metrics.response_time and metrics.response_time > self.slow_request_threshold:
                self._trigger_alert(
                    level=AlertLevel.WARNING,
                    message=f"Slow request detected: {metrics.method} {metrics.endpoint} took {metrics.response_time:.2f}s",
                    source="performance_monitoring",
                    details={
                        "request_id": request_id,
                        "response_time": metrics.response_time,
                        "threshold": self.slow_request_threshold,
                        "endpoint": metrics.endpoint,
                        "method": metrics.method
                    }
                )

            # Track failed requests
            if status_code >= 400:
                self.failed_requests += 1

            # Log metrics
            logger.info(
                f"Request {request_id}: {metrics.method} {metrics.endpoint} "
                f"completed in {metrics.response_time:.2f}s with status {status_code}"
            )

            # Remove from tracking after logging
            del self.request_metrics[request_id]

            # Check for high error rate and trigger alert if necessary
            self._check_error_rate()

    def _check_error_rate(self):
        """
        Check if the error rate exceeds the threshold and trigger an alert if needed
        """
        if self.total_requests > 10:  # Only check if we have enough requests to make a meaningful calculation
            current_error_rate = self.failed_requests / self.total_requests
            if current_error_rate > self.high_error_rate_threshold:
                self._trigger_alert(
                    level=AlertLevel.CRITICAL,
                    message=f"High error rate detected: {current_error_rate:.2%} (threshold: {self.high_error_rate_threshold:.2%})",
                    source="error_rate_monitoring",
                    details={
                        "error_rate": current_error_rate,
                        "threshold": self.high_error_rate_threshold,
                        "total_requests": self.total_requests,
                        "failed_requests": self.failed_requests
                    }
                )

    def _trigger_alert(self, level: AlertLevel, message: str, source: str, details: Optional[Dict[str, Any]] = None):
        """
        Trigger an alert with the given parameters

        Args:
            level: Alert level (INFO, WARNING, CRITICAL)
            message: Alert message
            source: Source of the alert
            details: Additional details about the alert
        """
        alert = Alert(
            level=level,
            message=message,
            timestamp=datetime.now(),
            source=source,
            details=details
        )

        self.alerts.append(alert)
        logger.log(
            logging.INFO if level == AlertLevel.INFO else
            logging.WARNING if level == AlertLevel.WARNING else
            logging.ERROR,
            f"ALERT [{level.value.upper()}]: {message}"
        )

        # Call registered alert callbacks
        for callback in self.alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                logger.error(f"Error in alert callback: {str(e)}")

    def add_alert_callback(self, callback: Callable[[Alert], None]):
        """
        Add a callback function to be called when alerts are triggered

        Args:
            callback: Function to call when alerts are triggered
        """
        self.alert_callbacks.append(callback)

    def get_alerts(self, level: Optional[AlertLevel] = None, limit: Optional[int] = None) -> List[Alert]:
        """
        Get alerts, optionally filtered by level and limited in number

        Args:
            level: Optional alert level to filter by
            limit: Optional maximum number of alerts to return

        Returns:
            List of alerts
        """
        alerts = self.alerts
        if level:
            alerts = [alert for alert in alerts if alert.level == level]

        if limit:
            alerts = alerts[-limit:]  # Return the most recent alerts

        return alerts

    def clear_alerts(self):
        """
        Clear all stored alerts
        """
        count = len(self.alerts)
        self.alerts.clear()
        logger.info(f"Cleared {count} alerts")

    def get_metrics_summary(self) -> Dict[str, Any]:
        """
        Get a summary of collected metrics

        Returns:
            Dictionary with metrics summary
        """
        uptime = time.time() - self.start_time
        success_rate = (
            (self.total_requests - self.failed_requests) / self.total_requests * 100
            if self.total_requests > 0 else 100
        )
        error_rate = (
            self.failed_requests / self.total_requests * 100
            if self.total_requests > 0 else 0
        )

        # Calculate average response time from completed requests
        completed_requests = [m for m in self.request_metrics.values() if m.response_time is not None]
        if completed_requests:
            avg_response_time = sum(m.response_time for m in completed_requests) / len(completed_requests)
        else:
            avg_response_time = 0.0

        return {
            "uptime_seconds": uptime,
            "total_requests": self.total_requests,
            "failed_requests": self.failed_requests,
            "success_rate_percent": success_rate,
            "error_rate_percent": error_rate,
            "current_active_requests": len(self.request_metrics),
            "average_response_time": avg_response_time,
            "alerts_count": len(self.alerts),
            "critical_alerts_count": len([a for a in self.alerts if a.level == AlertLevel.CRITICAL]),
            "warning_alerts_count": len([a for a in self.alerts if a.level == AlertLevel.WARNING])
        }

    def get_endpoint_metrics(self, endpoint: str) -> Dict[str, Any]:
        """
        Get metrics for a specific endpoint

        Args:
            endpoint: The endpoint to get metrics for

        Returns:
            Dictionary with endpoint-specific metrics
        """
        matching_requests = [
            metrics for metrics in self.request_metrics.values()
            if metrics.endpoint == endpoint
        ]

        if not matching_requests:
            return {
                "endpoint": endpoint,
                "total_requests": 0,
                "avg_response_time": 0,
                "min_response_time": 0,
                "max_response_time": 0
            }

        response_times = [
            metrics.response_time for metrics in matching_requests
            if metrics.response_time is not None
        ]

        if response_times:
            avg_response_time = sum(response_times) / len(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)
        else:
            avg_response_time = min_response_time = max_response_time = 0

        return {
            "endpoint": endpoint,
            "total_requests": len(matching_requests),
            "avg_response_time": avg_response_time,
            "min_response_time": min_response_time,
            "max_response_time": max_response_time
        }

    def reset_metrics(self):
        """
        Reset all collected metrics
        """
        self.total_requests = 0
        self.failed_requests = 0
        self.request_metrics.clear()
        self.start_time = time.time()
        logger.info("Metrics reset")


# Global monitoring service instance
monitoring_service = MonitoringService(
    slow_request_threshold=5.0,  # 5 seconds
    high_error_rate_threshold=0.1,  # 10% error rate
    high_response_time_threshold=3.0  # 3 seconds
)